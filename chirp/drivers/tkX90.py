# TK-690 support and improvements added in 2024 by Patrick Leiser, based on the work of:
# Copyright 2016-2024 Pavel Milanes CO7WT, <pavelmc@gmail.com>
#
# And with the help of Tom Hayward, who gently provided me with a driver he
# started and never finished for this radio.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import logging
import struct
import time

from chirp import chirp_common, directory, memmap, errors, util, bitwise
from textwrap import dedent
from chirp.settings import RadioSettingGroup, RadioSetting, \
    RadioSettingValueBoolean, RadioSettingValueList, \
    RadioSettingValueString, RadioSettingValueInteger, \
    RadioSettings

LOG = logging.getLogger(__name__)

# Note: the exported .dat files from the official KPG-44D software have a 
# metadata preable for the first 64 bytes, then contain the desired image
# to export to the radio itself (which is what chirp handles).

# To edit files in KPG-44D, create a blank file for the desired model, 
# then prepend the first 64 bytes of that file to the chirp img file


##### IMPORTANT MEM DATA #########################################
# This radios have a odd mem structure, it seems like you have to
# manage 3 memory sectors that we concatenate on just one big
# memmap, as follow
#
# Low memory (Main CPU)
#    0x0000 to 0x4000
# Mid memory (Unknown)
#    0x4000 to 0x4090
# High memory (Head)
#    0x4090 to 0x6090
###############################################################

MEM_FORMAT = """

//# seekto 0x0000;
struct {
    u8 unknown[16];
    u8 ch_name_length;
    u8 grp_name_length;
} settings;

#seekto 0x0400;
struct {
    u8 tot;                     // * 10 seconds, 30 sec increments
    u8 tot_pre_alert;           // seconds, off - 10, default 5
    u8 tot_rekey_time;          // seconds, off - 60, default off
    u8 tot_reset_time;          // seconds, off - 15, default off
    u8 unknown[4];
} group_settings[160];

#seekto 0x1480;
struct {
    u8 index;          // the index in the group_belong where this group start
    u8 length;         // how many channels are in this group
} group_limits[160];

#seekto 0x1600;
struct {
    u8 number;         // CH number relative to the group, 1-160
    u8 index;          // index relative to the memory space memory[index]
} group_belong[160];

#seekto 0x1800;
struct {
  lbcd rxfreq[4];       // 00-03
  lbcd txfreq[4];       // 04-07
  ul16 rxtone;
  ul16 txtone;
  u8 unknown0:1,
     power:1,            // power 0 = high, 1 = low
     beatshift:1,        // beat shift, 1 = on
     bcl:1,              // busy channel lockout, 1 = on
     pttid:1,            // ptt id, 1 = on
     optionSignalling:3; //off=0, 1=DTMF, 2,3,4 = "2-Tone 1,2,3"
  u8 unknown1:4,
     add:1,              // scan add, 1 = add
     unknown2:1,
     wide:1,             // Wide = 1, narrow = 0
     unknown4:1;
  u8 unknown5;
  u8 nose;
} memory[160];

#seekto 0x3DF0;
char poweron_msg[14];

#seekto 0x3E80;
struct {
  char line1[32];
  char line2[32];
} embeddedMessage;

#seekto 0x3ED0;
struct {
  u8 unknown10[10];
  char soft[6];
  u8 rid[10];
  u8 unknown11[6];
  u8 unknown12[11];
  char soft_ver[5];
} properties;

#seekto 0x4090;
struct {
  char name[16];
} grp_names[160];

//#seekto 0x4a90;
struct {
  char name[16];
} chs_names[160];

"""

MEM_SIZE = 0x6090  # 24720 bytes, 24,720 KiB
DAT_FILE_SIZE = 0x60E0
ACK_CMD = "\x06"
RX_BLOCK_SIZE_L = 128
MEM_LR = range(0x0380, 0x0400)
RX_BLOCK_SIZE_M = 16
MEM_MR = range(1, 10)
RX_BLOCK_SIZE_H = 32
MEM_HR = range(0, 0x2000, RX_BLOCK_SIZE_H)
EMPTY_L = "\xFF" * RX_BLOCK_SIZE_L
EMPTY_H = "\xFF" * RX_BLOCK_SIZE_H
POWER_LEVELS = [chirp_common.PowerLevel("High", watts=45),
                chirp_common.PowerLevel("Low", watts=5)]
MODES = ["NFM", "FM"]  # 12.5 / 25 Khz
VALID_CHARS = chirp_common.CHARSET_UPPER_NUMERIC + "()/\*@-+,.#_"
NAME_CHARS = 8
SKIP_VALUES = ["S", ""]
TONES = chirp_common.TONES
DTCS_CODES = chirp_common.DTCS_CODES

def _raw_recv(radio, amount):
    """Raw read from the radio device"""
    data = ""
    try:
        data = radio.pipe.read(amount)
    except Exception as e:
        raise errors.RadioError("Error reading data from radio: %s" % e)

    # DEBUG
    LOG.debug("<== (%d) bytes: %s" % (len(data), util.hexprint(data)))
    return data


def _raw_send(radio, data):
    """Raw send to the radio device"""
    try:
        radio.pipe.flush()
        radio.pipe.write(data)
        # DEBUG
        LOG.debug("==> (%d) bytes: %s" % (len(data), util.hexprint(data)))
    except:
        raise errors.RadioError("Error sending data to radio")


def _close_radio(radio):
    """Get the radio out of program mode"""
    _raw_send(radio, "E")


def _checksum(data):
    """the radio block checksum algorithm"""
    cs = 0
    for byte in data:
            cs += ord(byte)
    return cs % 256


def _send(radio, frame):
    """Generic send data to the radio"""
    _raw_send(radio, frame)


def _make_framel(cmd, addr):
    """Pack the info in the format it likes"""
    # x52 x0F (x0380-x0400)
    return struct.pack(">BBH", ord(cmd), 0x0F, addr)


def _make_framem(cmd, addr):
    """Pack the info in the format it likes"""
    # x54 x0F (x00-x0A)
    return struct.pack(">BBB", ord(cmd), 0x0F, addr)


def _make_frameh(cmd, addr):
    """Pack the info in the format it likes"""
    # x53 x8F (x0000-x2000) x20
    return struct.pack(">BBHB", ord(cmd), 0x8F, addr, RX_BLOCK_SIZE_H)


def _handshake(radio, msg="", full=True):
    """Make a full handshake"""
    if full is True:
        # send ACK
        _raw_send(radio, ACK_CMD)

    # receive ACK
    ack = _raw_recv(radio, 1)

    # check ACK
    if ack != ACK_CMD:
        _close_radio(radio)
        mesg = "Handshake failed, got ack: '0x%02x': " % ord(ack)
        mesg += msg
        # DEBUG
        LOG.debug(mesg)
        raise errors.RadioError(mesg)


def _recvl(radio):
    """Receive low data block from the radio, 130 or 2 bytes"""
    rxdata = _raw_recv(radio, 2)
    if rxdata == "\x5A\xFF":
        # when the RX block has 2 bytes and the paylod+CS is \x5A\xFF
        # then the block is all \xFF
        _handshake(radio, "short block")
        return False
    rxdata += _raw_recv(radio, RX_BLOCK_SIZE_L)
    if len(rxdata) == RX_BLOCK_SIZE_L + 2 and rxdata[0] == "W":
        # Data block is W + Data(128) + CS
        rcs = ord(rxdata[-1])
        data = rxdata[1:-1]
        ccs = _checksum(data)

        if rcs != ccs:
            msg = "Block Checksum Error! real %02x, calculated %02x" % \
                  (rcs, ccs)
            LOG.error(msg)
            _handshake(radio)
            _close_radio(radio)
            raise errors.RadioError(msg)

        _handshake(radio, "After checksum in Low Mem")
        return data
    else:
        raise errors.RadioError("Radio send an answer we don't understand")


def _recvh(radio):
    """Receive high data from the radio, 35 or 4 bytes"""
    rxdata = _raw_recv(radio, 4)
    # There are two valid options, the first byte is the content
    if len(rxdata) == 4 and rxdata[0] == "\x5B" and rxdata[3] == "\xFF":
        # 4 bytes, x5B = empty; payload = xFF (block is all xFF)
        _handshake(radio, "Short block in High Mem")
        return False
    rxdata += _raw_recv(radio, RX_BLOCK_SIZE_H - 1)
    if len(rxdata) == RX_BLOCK_SIZE_H + 3 and rxdata[0] == "\x58":
        # 35 bytes, x58 + address(2) + data(32), no checksum
        data = rxdata[3:]
        _handshake(radio, "After data in High Mem")
        return data
    else:
        _close_radio(radio)
        raise errors.RadioError("Radio send a answer we don't understand")


def _open_radio(radio):
    """Open the radio into program mode and check if it's the correct model"""
    radio.pipe.baudrate = 9600
    radio.pipe.timeout = 1.0

    LOG.debug("Starting program mode.")

    _raw_send(radio, "PROGRAM")
    ack = _raw_recv(radio, 1024)
    while ack:
        if ack[0] == ACK_CMD:
            break
        ack = ack[ack.index('\xff')+1:]
    else:
        radio.pipe.write("E")
        raise errors.RadioError("Radio didn't acknowledge program mode.")

    # DEBUG
    LOG.debug("Received correct ACK to the MAGIC, send ID query.")
    LOG.info("Radio entered Program mode.")

    _raw_send(radio, "\x02\x0F")
    rid = _raw_recv(radio, 10)

    if not rid.startswith(radio.TYPE):
        # bad response, properly close the radio before exception
        _close_radio(radio)

        # DEBUG
        LOG.debug("Incorrect model ID:")
        LOG.debug(util.hexprint(rid))

        raise errors.RadioError(
            "Incorrect model ID, got %s, it not contains %s" %
            (rid.strip("\xff"), radio.TYPE))

    # DEBUG
    LOG.info("Positive ID on radio.")
    LOG.debug("Full ident string is:\n%s" % util.hexprint(rid))

    _handshake(radio)


def do_download(radio):
    """ The download function """
    # UI progress
    status = chirp_common.Status()
    status.cur = 0
    status.max = MEM_SIZE
    status.msg = ""
    radio.status_fn(status)

    # open the radio
    _open_radio(radio)

    # initialize variables
    data = ""
    bar = 0

    # DEBUG
    LOG.debug("Starting the download from radio.")

    # speed up the reading for this stage
    # radio.pipe.timeout = (0.08)  # never below 0.08 or you will get errors

    for addr in MEM_LR:
        _send(radio, _make_framel("R", addr))
        d = _recvl(radio)
        # if empty block, return false = full of xFF
        if d == False:
            d = EMPTY_L

        # aggregate the data
        data += d

        # UI update
        bar += RX_BLOCK_SIZE_L
        status.cur = bar
        status.msg = "Cloning from Main MCU (Low mem)..."
        radio.status_fn(status)

    # DEBUG
    LOG.debug("Main MCU (low) mem received")

    # speed up the reading for this stage
    # radio.pipe.timeout = (0.04)  # never below 0.04 or you will get errors

    for addr in MEM_MR:
        _send(radio, _make_framem("T", addr))
        d = _raw_recv(radio, 17)

        if len(d) != 17 :
            raise errors.RadioError(
                "Problem receiving short block %d on mid mem" % addr)

        # Aggregate data ans hansdhake
        data += d[1:]
        _handshake(radio, "Middle mem ack error")

        # UI update
        bar += RX_BLOCK_SIZE_M
        status.cur = bar
        status.msg = "Cloning from 'unknown' (mid mem)..."
        radio.status_fn(status)

    # DEBUG
    LOG.debug("Middle mem received.")

    # speed up the reading for this stage
    # radio.pipe.timeout = (0.08)  # never below 0.08 or you will get errors
    for addr in MEM_HR:
        _send(radio, _make_frameh("S", addr))
        # FIXME: empty blocks are short and always expire timeout
        d = _recvh(radio)
        # if empty block, return false = full of xFF
        if d == False:
            d = EMPTY_H

        # aggregate the data
        data += d

        # UI update
        bar += RX_BLOCK_SIZE_H
        status.cur = bar
        status.msg = "Cloning from Head (High mem)..."
        radio.status_fn(status)

    # DEBUG
    LOG.debug("Head (high) mem received")
    LOG.info("Full Memory received ok.")

    _close_radio(radio)
    return memmap.MemoryMapBytes(data)


def do_upload(radio):
    """ The upload function """
    # UI progress
    status = chirp_common.Status()
    status.cur = 0
    status.max = MEM_SIZE
    status.msg = "Getting the radio into program mode."
    radio.status_fn(status)
    # open the radio
    _open_radio(radio)

    # initialize variables
    bar = 0
    img = radio.get_mmap()

    # DEBUG
    LOG.debug("Starting the upload to the radio")

    for addr in MEM_LR:
        # this is the data to write
        data = img[bar:bar + RX_BLOCK_SIZE_L]
        # this is the full packet to send
        sdata = ""

        # flag
        short = False

        # building the data to send
        if data == EMPTY_L:
            # empty block
            sdata = _make_framel("Z", addr) + "\xFF"
            short = True
        else:
            # normal
            cs = _checksum(data)
            sdata = _make_framel("W", addr) + data + chr(cs)

        # send the data
        _send(radio, sdata)

        # DEBUG
        LOG.debug("Sended memmap pos 0x%04x" % bar)

        # slow MCU
        # time.sleep(0.15)

        # check ack
        msg = "Bad ACK on low block %04x" % addr
        _handshake(radio, msg, False)

        # UI Update
        bar += RX_BLOCK_SIZE_L
        status.cur = bar
        status.msg = "Cloning to Main MCU (Low mem)..."
        radio.status_fn(status)

    # DEBUG
    LOG.debug("Main MCU (low) mem received")

    # speed up the reading for this stage
    # radio.pipe.timeout = (0.04)  # never below 0.04 or you will get errors
    for addr in MEM_MR:
        # this is the data to write
        data = img[bar:bar + RX_BLOCK_SIZE_M]
        sdata = _make_framem("Y", addr) + "\x00" + data

        # send it
        _send(radio, sdata)

        # DEBUG
        LOG.debug("Sent memmap pos 0x%04x" % bar)

        # slow MCU
        # time.sleep(0.2)

        # check ack
        msg = "Bad ACK on mid block %04x" % addr
        _handshake(radio, msg, not short)

        # UI Update
        bar += RX_BLOCK_SIZE_M
        status.cur = bar
        status.msg = "Cloning from middle mem..."
        radio.status_fn(status)

    # DEBUG
    LOG.debug("Middle mem received")

    for addr in MEM_HR:
        # this is the data to write
        data = img[bar:bar + RX_BLOCK_SIZE_H]
        # this is the full packet to send
        sdata = ""

        # building the data to send
        if data == EMPTY_H:
            # empty block
            sdata = _make_frameh("[", addr) + "\xFF"
        else:
            # normal
            sdata = _make_frameh("X", addr) + data

        # send the data
        _send(radio, sdata)

        # DEBUG
        LOG.debug("Sended memmap pos 0x%04x" % bar)

        # slow MCU
        # time.sleep(0.15)

        # check ack
        msg = "Bad ACK on low block %04x" % addr
        _handshake(radio, msg, False)

        # UI Update
        bar += RX_BLOCK_SIZE_H
        status.cur = bar
        status.msg = "Cloning to Head MCU (high mem)..."
        radio.status_fn(status)

    # DEBUG
    LOG.debug("Head (high) mem received")
    # DEBUG
    LOG.info("Upload finished")


    _close_radio(radio)


def model_match(cls, data, rid_index=0x03EE0):
    """Match the opened/downloaded image to the correct version"""
    rid = _get_rid(data, rid_index)

    # DEBUG
    print("Radio ID is:")
    print(util.hexprint(rid))
    print(cls.VARIANTS)
    if (rid in cls.VARIANTS):
        print("File/Data match for ID.")
        return True
    else:
        print("BAD File/Data match.")
        return False


def _get_rid(data, index=0x03EE0):
    """Get the radio ID string from a mem string"""
    return data[index:index+6]


class Kenwoodx90BankModel(chirp_common.BankModel):
    """Testing the bank model on kenwood"""
    channelAlwaysHasBank = True
    def get_num_mappings(self):
        return self._radio._num_banks

    def get_mappings(self):
        banks = []
        for i in range(0, self._radio._num_banks):
            # display group number
            bindex = i + 1
            # display name of the channel
            gname = "%03i" % bindex
            # assign the channel
            bank = self._radio._bclass(self, i, gname )
            bank.index = i
            banks.append(bank)
        return banks

    def add_memory_to_mapping(self, memory, bank):
        self._radio._set_bank(memory.number, bank.index)

    def remove_memory_from_mapping(self, memory, bank):
        if self._radio._get_bank(memory.number) != bank.index:
            raise Exception("Memory %i not in bank %s. Cannot remove." %
                            (memory.number, bank))

        # We can't "Remove" it for good the kenwood paradigm don't allow it
        # instead we move it to bank 0
        self._radio._set_bank(memory.number, 0)

    def get_mapping_memories(self, bank):
        memories = []
        for i in range(0, self._radio._upper):
            if self._radio._get_bank(i) == bank.index:
                memories.append(self._radio.get_memory(i))
        return memories

    def get_memory_mappings(self, memory):
        index = self._radio._get_bank(memory.number)
        if index is None:
            return []
        return [self.get_mappings()[index]]


class memBank(chirp_common.Bank):
    """A bank model for kenwood"""
    # Integral index of the bank, not to be confused with per-memory
    # bank indexes
    index = 0


class Kenwoodx90(chirp_common.CloneModeRadio, chirp_common.ExperimentalRadio):
    """Kenwood TK-790 radio base class"""
    VENDOR = "Kenwood"
    BAUD_RATE = 9600
    VARIANT = ""
    MODEL = ""
    NAME_LENGTH = 6
    # others
    _memsize = MEM_SIZE
    _range = [136000000, 162000000]
    _upper = 160
    _banks = dict()
    _num_banks = 160
    _bclass = memBank
    _kind = ""
    FORMATS = [directory.register_format('Kenwood KPG-44D', '*.dat')]


    @classmethod
    def get_prompts(cls):
        rp = chirp_common.RadioPrompts()
        rp.experimental = \
            ('========== PLEASE READ THIS NOTICE FULLY. ===============\n'
             '\n'
             'This driver is experimental and for personal use only. '
             'Please do a backup with the KPG44 software BEFORE using it '
             'to have a way of restoring the radio\'s features if something '
             'goes wrong.\n'
             '\n'
             'By now just channel management and basic banks support is '
             'included, the driver still in development, so any success '
             'or failure report are welcomed. (keep reading)\n'
             '\n'
             'If you uses GROUP names this driver will ignore that and can '
             'fail, that feature is in development\n'
             '\n'
             'It\'s known that there are some special firmware version radios '
             'specifically for California, this driver may not work with that '
             'radios, if so please report to the developer at: '
             'co7wt@frcuba.co.cu or via the chirp web interface\n'

             )
        rp.pre_download = _(dedent("""\
            Follow this instructions to read from your radio info:
            1 - Turn off your radio
            2 - Connect your interface cable
            3 - Turn on your radio (unblock it if password protected)
            4 - Do the download of your radio data

            The structure of the radio data is different from normal radios
            so the download will take up to 2+ minutes.

            """))
        rp.pre_upload = _(dedent("""\
            Follow this instructions to write to your radio info:
            1 - Turn off your radio
            2 - Connect your interface cable
            3 - Turn on your radio (unblock it if password protected)
            4 - Do the upload of your radio data

            The structure of the radio data is different from normal radios
            so the upload will take up to 2+ minutes.

            """))
        return rp

    def get_features(self):
        """Return information about this radio's features"""
        rf = chirp_common.RadioFeatures()
        rf.has_settings = False   #True
        rf.has_bank = True
        rf.has_tuning_step = False
        rf.has_name = True
        rf.has_offset = True
        rf.has_mode = True
        rf.has_dtcs = True
        rf.has_rx_dtcs = True
        rf.has_dtcs_polarity = True
        rf.has_ctone = True
        rf.has_cross = True
        rf.valid_modes = MODES
        rf.valid_duplexes = ["", "-", "+", "off"]
        rf.valid_tmodes = ['', 'Tone', 'TSQL', 'DTCS', 'Cross']
        rf.valid_cross_modes = [
            "Tone->Tone",
            "DTCS->",
            "->DTCS",
            "Tone->DTCS",
            "DTCS->Tone",
            "->Tone",
            "DTCS->DTCS"]
        rf.valid_power_levels = POWER_LEVELS
        rf.valid_characters = VALID_CHARS
        rf.valid_skips = SKIP_VALUES
        rf.valid_dtcs_codes = DTCS_CODES
        rf.valid_bands = [self._range]
        rf.valid_name_length = NAME_CHARS
        rf.memory_bounds = (1, self._upper)
        return rf

    def _fill(self, offset, data):
        """Fill an specified area of the memmap with the passed data"""
        for addr in range(0, len(data)):
            self._mmap[offset + addr] = data[addr]

    def _prep_data(self):
        """Prepare the areas in the memmap to do a consistend write
        it has to make an update on the x1600 area with banks and channel
        info; other in the x1000 with banks and channel counts
        and a last one in x7000 with flog data"""
        rchs = 0
        data = dict()

        # sorting the data
        for ch in range(0, self._upper):
            mem = self._memobj.memory[ch]
            bnumb = int(mem.bnumb)
            bank = int(mem.bank)
            if bnumb != 255 and (bank != 255 and bank != 0):
                try:
                    data[bank].append(ch)
                except:
                    data[bank] = list()
                    data[bank].append(ch)
                data[bank].sort()
                # counting the real channels
                rchs = rchs + 1

        # updating the channel/bank count
        self._memobj.settings.channels = rchs
        self._chs_progs = rchs
        self._memobj.settings.banks = len(data)

        # building the data for the memmap
        fdata = ""

        for k, v in data.iteritems():
            # posible bad data
            if k == 0:
                k = 1
                raise errors.InvalidValueError(
                    "Invalid bank value '%k', bad data in the image? \
                    Triying to fix this, review your bank data!" % k)
            c = 1
            for i in v:
                fdata += chr(k) + chr(c) + chr(k - 1) + chr(i)
                c = c + 1

        # fill to match a full 256 bytes block
        fdata += (len(fdata) % 256) * "\xFF"

        # updating the data in the memmap [x300]
        self._fill(0x300, fdata)

        # update the info in x1000; it has 2 bytes with
        # x00 = bank , x01 = bank's channel count
        # the rest of the 14 bytes are \xff
        bdata = ""
        for i in range(1, len(data) + 1):
            line = chr(i) + chr(len(data[i]))
            line += "\xff" * 14
            bdata += line

        # fill to match a full 256 bytes block
        bdata += (256 - (len(bdata)) % 256) * "\xFF"

        # fill to match the whole area
        bdata += (16 - len(bdata) / 256) * EMPTY_BLOCK

        # updating the data in the memmap [x1000]
        self._fill(0x1000, bdata)

        # DTMF id for each channel, 5 bytes lbcd at x7000
        # ############## TODO ###################
        fldata = "\x00\xf0\xff\xff\xff" * self._chs_progs + \
            "\xff" * (5 * (self._upper - self._chs_progs))

        # write it
        # updating the data in the memmap [x7000]
        self._fill(0x7000, fldata)

    def _set_variant(self):
        """Select and set the correct variables for the class acording
        to the correct variant of the radio, and other runtime data"""
        rid = _get_rid(self.get_mmap())

        # indentify the radio variant and set the enviroment to it's values
        try:
            self._upper, low, high, self._kind = self.VARIANTS[rid]
            self._range = [low * 1000000, high * 1000000]

            # put the VARIANT in the class, clean the model / CHs / Type
            # in the same layout as the KPG program
            self._VARIANT = self.MODEL + " [" + str(self._upper) + "CH]: "
            self._VARIANT += self._kind + ", " + str(self._range[0]/1000000) + "-"
            self._VARIANT += str(self._range[1]/1000000) + " Mhz"

            # DEBUG
            LOG.info("Radio variant: %s" % self._VARIANT)

        except KeyError:
            LOG.debug("Wrong Kenwood radio, ID or unknown variant")
            LOG.debug(util.hexprint(rid))
            raise errors.RadioError(
                "Wrong Kenwood radio, ID or unknown variant, see LOG output.")

        # the channel name length is a variable in the radio settings
        NAME_CHARS = int(self._memobj.settings.ch_name_length)

    def sync_in(self):
        """Do a download of the radio eeprom"""
        self._mmap = do_download(self)
        self.process_mmap()

    def sync_out(self):
        """Do an upload to the radio eeprom"""
        try:
            do_upload(self)
        except errors.RadioError:
            raise
        except Exception as e:
            raise errors.RadioError("Failed to communicate with radio: %s" % e)

    def _get_bank_struct(self):
        """Parse the bank data in the mem into the self.bank variable"""
        # Variables
        gl = self._memobj.group_limits
        gb = self._memobj.group_belong
        bank_count = 0

        for bg in gl:
            # check for empty banks
            if bg.index == 255 and bg.length == 255:
                self._banks[bank_count] = list()
                # increment the bank count
                bank_count += 1
                continue

            for i in range(0, bg.length):
                # bank inside this channel
                position = bg.index + i
                index = int(gb[position].index)

                try:
                    self._banks[bank_count].append(index)
                except KeyError:
                    self._banks[bank_count] = list()
                    self._banks[bank_count].append(index)

            # increment the bank count
            bank_count += 1

    def process_mmap(self):
        """Process the memory object"""
        # load the memobj
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

        # to ser the vars on the class to the correct ones
        self._set_variant()

        # load the bank data
        self._get_bank_struct()

    def load_mmap(self, filename):
        print('loading'+filename) #LOG.info
        if filename.lower().endswith('.dat'):
            with  open(filename, "rb") as f:
                self._datHeaderMmap = memmap.MemoryMapBytes(f.read(0x40))
                #f.seek(0x40)
                self._mmap = memmap.MemoryMapBytes(f.read())
                print('loaded KPG-44D dat file at offset 0x40') #LOG.info
            self.process_mmap()
        else:
            self._datHeaderMmap = None
            chirp_common.CloneModeRadio.load_mmap(self, filename)

    def save_mmap(self, filename):
        print("saving as"+filename)
        if filename.lower().endswith('.dat'):
            with open(filename, 'wb') as f:
                datHeader = self._prep_dat_header()
                f.write(datHeader.get_packed())
                f.write(self._mmap.get_packed())
                LOG.info("Wrote KPG-44D dat file")
        else:
            chirp_common.CloneModeRadio.save_mmap(self, filename)

    def _prep_dat_header(self):
        if self._datHeaderMmap is not None: #if dat header imported with file
            return self._datHeaderMmap
        #otherwise build our own header
        datHeaderMap = memmap.MemoryMapBytes(bytes([255]*0x40))
        softwareName = self._mmap.get(0x3EDA, 6)
        softwareVer = self._mmap.get(0x3EFB, 5)
        rid = self._mmap.get(0x3EE0, 10)
        datHeaderMap.set(0x00, softwareName)
        datHeaderMap.set(0x0A, softwareVer)
        datHeaderMap.set(0x0F, rid)
        print(datHeaderMap.printable())
        return datHeaderMap

    def get_raw_memory(self, number):
        """Return a raw representation of the memory object, which
        is very helpful for development"""
        return repr(self._memobj.memory[number])

    def _decode_tone(self, val):
        """Parse the tone data to decode from mem, it returns:
        Mode (''|DTCS|Tone), Value (None|###), Polarity (None,N,R)"""
        val = int(val)
        if val == 65535:
            return '', None, None
        elif val >= 0x2800:
            code = int("%03o" % (val & 0x07FF))
            pol = (val & 0x8000) and "R" or "N"
            return 'DTCS', code, pol
        else:
            a = val / 10.0
            return 'Tone', a, None

    def _encode_tone(self, memval, mode, value, pol):
        """Parse the tone data to encode from UI to mem"""
        if mode == '':
            memval.set_raw(b"\xff\xff")
        elif mode == 'Tone':
            memval.set_value(int(value * 10))
        elif mode == 'DTCS':
            val = int("%i" % value, 8) + 0x2800
            if pol == "R":
                val += 0xA000
            memval.set_value(val)
        else:
            raise Exception("Internal error: invalid mode `%s'" % mode)

    def get_memory(self, number):
        """Get the mem representation from the radio image"""

        # Create a high-level memory object to return to the UI
        mem = chirp_common.Memory()

        # Memory number
        mem.number = number

        # use mem_index instead of pure number index, but must track unallocated memory locations
        _mem_index = self._get_mem_index(number)
        if _mem_index == 0xFF:
            mem.empty = True
            return mem
        _mem = self._memobj.memory[_mem_index]
        _chs_names = self._memobj.chs_names[_mem_index+1]



        if _mem.get_raw()[0] == 0xFF:
            mem.empty = True
            return mem

        # Freq and offset
        mem.freq = int(_mem.rxfreq) * 10
        # tx freq can be blank
        if _mem.get_raw()[4] == 0xFF:
            # TX freq not set
            mem.offset = 0
            mem.duplex = "off"
        else:
            # TX feq set
            offset = (int(_mem.txfreq) * 10) - mem.freq
            if offset < 0:
                mem.offset = abs(offset)
                mem.duplex = "-"
            elif offset > 0:
                mem.offset = offset
                mem.duplex = "+"
            else:
                mem.offset = 0

        # name TAG of the channel
        mem.name = str(_chs_names.name).rstrip(" ")[:NAME_CHARS + 1]

        # power (0 = high, 1 = low)
        mem.power = POWER_LEVELS[int(_mem.power)]

        # wide/marrow
        mem.mode = MODES[int(_mem.wide)]

        # skip
        mem.skip = SKIP_VALUES[int(_mem.add)]

        # tone data
        rxtone = txtone = None
        txtone = self._decode_tone(_mem.txtone)
        rxtone = self._decode_tone(_mem.rxtone)
        chirp_common.split_tone_decode(mem, txtone, rxtone)

        # Extra
        mem.extra = RadioSettingGroup("extra", "Extra")

        bcl = RadioSetting("bcl", "Busy channel lockout",
                          RadioSettingValueBoolean(bool(_mem.bcl)))
        mem.extra.append(bcl)

        pttid = RadioSetting("pttid", "PTT ID",
                          RadioSettingValueBoolean(bool(_mem.pttid)))
        mem.extra.append(pttid)

        beat = RadioSetting("beatshift", "Beat Shift",
                          RadioSettingValueBoolean(bool(_mem.beatshift)))
        mem.extra.append(beat)

        return mem

    def set_memory(self, mem):
        """Set the memory data in the eeprom img from the UI"""
        # get the eprom representation of this channel
        _mem_index = self._get_mem_index(mem.number)
        if _mem_index == 0xFF:
            print(mem)
            print("allocating new memory location")
            for i in range (0, 160):
                if self._memobj.memory[i].get_raw()[0] == 0xFF: #check for empty mem location
                    i += 1
                    _mem_index = i
                    self._memobj.group_belong[mem.number].index=i
                    #TODO set group_belong.number
                    print("found empty memory location: " + str(i))
                    break
        _mem = self._memobj.memory[_mem_index]
        _ch_name = self._memobj.chs_names[_mem_index]

        # if empty memory
        if mem.empty:
            # the channel it self
            _mem.set_raw(bytes([0xFF] * 16))

            # the name tag
            for byte in _ch_name.name:
                byte.set_raw(b'\xFF')

            # delete it from the banks
            self._del_channel_from_bank(mem.number)

            return

        # frequency
        _mem.rxfreq = mem.freq / 10

        # duplex
        if mem.duplex == "+":
            _mem.txfreq = (mem.freq + mem.offset) / 10
        elif mem.duplex == "-":
            _mem.txfreq = (mem.freq - mem.offset) / 10
        elif mem.duplex == "off":
            for byte in _mem.txfreq:
                byte.set_raw(b'\xFF')
                
        else:
            _mem.txfreq = mem.freq / 10

        # tone data
        ((txmode, txtone, txpol), (rxmode, rxtone, rxpol)) = \
            chirp_common.split_tone_encode(mem)
        self._encode_tone(_mem.txtone, txmode, txtone, txpol)
        self._encode_tone(_mem.rxtone, rxmode, rxtone, rxpol)

        # name TAG of the channel
        _ch_name.name = str(mem.name).ljust(16, " ")

        # power, # default power is low  (0 = high, 1 = low)
        _mem.power = 0 if mem.power is None else POWER_LEVELS.index(mem.power)

        # wide/marrow
        _mem.wide = MODES.index(mem.mode)

        # scan add property
        _mem.add = SKIP_VALUES.index(mem.skip)

        # reseting unknowns, this have to be set !?!?!?!?
        _mem.nose.set_raw(b"\xFE")

        # extra settings
        if len(mem.extra) > 0:
            # there are setting, parse
            for setting in mem.extra:
                setattr(_mem, setting.get_name(), bool(setting.value))
        else:
            msg = "Channel #%d has no extra data, loading defaults" % \
                  int(mem.number - 1)
            LOG.info(msg)
            # there is no extra settings, load defaults
            _mem.bcl = 0
            _mem.pttid = 0
            _mem.beatshift = 0
            # unknowns
            _mem.unknown0 = 0
            _mem.unknown1 = 0

        # it's new and we need to update it's bank state?
        b = self._get_bank(mem.number)
        if b == None:
            self._set_bank(mem.number, 1)
        print(mem)
        print(_mem)
        #print(self._mmap.printable())
        return mem

    @classmethod
    def match_model(cls, filedata, filename):
        match_size = False
        match_model = False

        if filename.lower().endswith('.dat'):
            if (len(filedata) == DAT_FILE_SIZE):
                match_size = True
            #.dat metadata specifies model near start of file
            match_model = model_match(cls, filedata, 0x000F)
        # testing the file data size
        #print(len(filedata))
        #print(MEM_SIZE)
        #TODO figure out the cause of this size discrepancy
        if (len(filedata) == MEM_SIZE) or (len(filedata) == MEM_SIZE + 16):
            match_size = True
            LOG.info("File match for size")

        # testing the firmware model fingerprint
        match_model = match_model | model_match(cls, filedata)

        if match_size and match_model:
            return True
        else:
            return False

    def _get_mem_index(self, index):
        return(self._memobj.group_belong[index-1].index)

    def get_bank_model(self):
        """Pass the bank model to the UI part"""
        rf = self.get_features()
        if rf.has_bank is True:
            return Kenwoodx90BankModel(self)
        else:
            return None

    def _get_bank(self, loc):
        """Get the bank data for a specific channel"""
        _mem_index = self._get_mem_index(loc)
        for k in self._banks:
            if (_mem_index) in self._banks[k]:
                # DEBUG
                print("Channel %d is in bank %d" % (loc, k))
                return k

        return None

    def _set_bank(self, loc, bank=0):
        """Set the bank data for a specific channel"""
        # it's on the same bank?
        b = self._get_bank(loc)
        if b == bank:
            return

        # it's already asigned, delete from there
        if b != None:
            self._del_channel_from_bank(loc, bank)

        # adding it
        # DEBUG
        LOG.debug("Loc %d is not in bank %d, adding it" % (loc, bank))
        self._banks[bank].append(self._get_mem_index(loc))

        # if the update was successful update in the memmap
        self._update_bank_memmap()

    def _del_channel_from_bank(self, loc, bank=None):
        """Remove a channel from a bank, if no bank is specified we search
        from where it is"""
        print(self._banks)
        # some times we need to just erase it not knowing where it's
        # if so,
        if bank == None:
            bank = self._get_bank(loc)

        # remove it
        self._banks[bank].pop(self._banks[bank].index(self._get_mem_index(loc)))

        ## check if the banks got empty to erase it
        #if len(self._banks[bank]) == 0:
            #self._banks.pop(bank)

        # if the delete was successful update in the memmap
        self._update_bank_memmap()

    def _update_bank_memmap(self):
        """This function is called whatever a change is made to a channel
        or a bank, to update the memmap with the changes that was made"""

        bl = b""
        bb = b""

        # group_belong index
        gbi = 0
        for bank in self._banks:
            # check for empty banks
            if len(self._banks[bank]) == 0:
                bl += b"\xff\xff"
                continue

            # channel index inside the bank, starting at 1
            # aka channel in group index
            cgi = 1
            for channel in range(0, len(self._banks[bank])):
                # update bb
                bb += bytes(cgi) + bytes(self._banks[bank][channel])
                # set the group limitst for this group
                if cgi == 1:
                    bl += bytes(gbi) + bytes(len(self._banks[bank]))

                # increments
                gbi += 1
                cgi += 1

        # fill the gaps before write it
        bb += b"\xff" * 2 * int(self._num_banks - len(bb) / 2)
        bl += b"\xff" * 2 * int(self._num_banks - len(bl) / 2)

        # update the memmap
        self._fill(0x1480, bl)
        self._fill(0x1600, bb)


@directory.register
class TK790_Radios(Kenwoodx90):
    """Kenwood TK-790 K/K2"""
    MODEL = "TK-790"
    TYPE = "M0790"
    VARIANTS = {
        b"M0790\x04": (160, 144, 174, "K"),  # se note below
        b"M0790\x05": (160, 136, 156, "K2")
    }

@directory.register
class TK690_Radios(Kenwoodx90):
    """Kenwood TK-690 """
    MODEL = "TK-690"
    TYPE = "M0690"
    VARIANTS = {
        b"M0690\x01": (160, 28, 37, "K"),  # see note below
        b"M0690\x02": (160, 35, 43, "K2"),
        b"M0690\x03": (160, 136, 156, "K3")
    }

    # Note:
    # These radios originaly are constrained to some band segments but the
    # original software doesn't care much about it, so in order to match a
    # feature many will miss from the factory software and to help
    # the use of this radios in the ham bands I'm expanding the range
    # of the "K" version of the TK-790 from 148 to 144, as well as the
    # range of the TK-690 "F1" (from 29.7 to 28) and "F3" (from 50 to 54)
    # versions (note that F3 also needs physical modifications for use with
    # Ham bands)


