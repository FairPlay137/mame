// license:BSD-3-Clause
// copyright-holders:FairPlay137,wtvemac

/****************************************************************************************
 *
 * WebTV FCS (1996)
 * 
 * The WebTV line of products was a set of thin clients meant to make the emerging-at-
 * -the-time Internet accessible to those who weren't as familiar with computers,
 * adapting websites for display on television sets, simplifying the control scheme to
 * work with just a remote control and an optional keyboard, and optimizing them to work
 * smoothly on low-cost hardware. Later on in its life, it was rebranded as MSN TV.
 * 
 * FCS, shorthand for First Customer Ship, was the first generation of WebTV hardware.
 * Its ASIC, known as SPOT or FIDO, is much simpler than SOLO.
 * 
 * A typical retail configuration uses a MIPS IDT R4640 clocked at 112MHz, with 2MB of
 * on-board RAM, 2MB of flash memory (using two 16-bit 1MB AMD 29F800B chips) for the
 * updatable software (referred to AppROMs officially), and 2MB of mask ROM (across two
 * 1MB chips).
 * 
 * This driver would not have been possible without the efforts of the WebTV community
 * to preserve technical specifications, as well as the various reverse-engineering
 * efforts that were made.
 * 
 * Currently unimplemented:
 * - IR keyboard and remote emulation. This seems to be handled by a PIC16CR54AT on
 *   board. No dump is currently available for this chip, so for the meantime it will
 *   need to be emulated at a high level.
 * - SmartCard emulation. This would involve turning the SmartCard slot into its own
 *   slot device.
 * - WebTV Port emulation. On the right side of all WebTV FCS units is a port which was
 *   used for add-ons. The only add-on which ever saw a release was an adapter which
 *   allowed parallel port printers to work on this generation of WebTV hardware.
 *   However, there was another WebTV Port device, known as Pekoe, which was used
 *   internally for debugging.
 * 
 * Planned additions:
 * - 4MB RAM configuration for PAL units - later software revisions are VERY unstable in
 *   PAL mode and prone to running out of RAM.
 * 
 ****************************************************************************************/

#include "emu.h"

#include "bus/pc_kbd/keyboards.h"
#include "cpu/mips/mips3.h"
#include "machine/ds2401.h"
#include "machine/intelfsh.h"
#include "spot_asic.h"

#include "webtv.lh"

#include "main.h"
#include "screen.h"

// The system clock is used to drive the SPOT ASIC, drive the CPU and is used to calculate the audio clock.
// 56.448MHz was probably chosen because it's a multiple of the audio clock? (44100Hz)
constexpr XTAL     SYSCLOCK         = XTAL(56'448'000);
constexpr uint16_t RAM_FLASHER_SIZE = 0x100;

class webtv1_state : public driver_device
{
public:
	// constructor
	webtv1_state(const machine_config& mconfig, device_type type, const char* tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_spotasic(*this, "spot"),
		m_serial_id(*this, "serial_id"),
		m_nvram(*this, "nvram"),
		m_flash0(*this, "bank0_flash0"), // labeled U0501, contains upper bits
		m_flash1(*this, "bank0_flash1")  // labeled U0502, contains lower bits
	{ }

	void webtv1_base(machine_config& config);
	void webtv1_sony(machine_config& config);
	void webtv1_philips(machine_config& config);
	void webtv1_bfe(machine_config& config);
	void webtv1_japan(machine_config& config);

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;

private:
	required_device<mips3_device> m_maincpu;
	required_device<spot_asic_device> m_spotasic;
	required_device<ds2401_device> m_serial_id;
	required_device<i2cmem_device> m_nvram;

	required_device<intelfsh16_device> m_flash0;
	required_device<intelfsh16_device> m_flash1;

	uint8_t ram_flasher[RAM_FLASHER_SIZE];

	uint8_t m_manufacturer_code; // temporary; will determine remote manufacturer

	void bank0_flash_w(offs_t offset, uint32_t data);
	uint32_t bank0_flash_r(offs_t offset);
	uint8_t ram_flasher_r(offs_t offset);
	void ram_flasher_w(offs_t offset, uint8_t data);

	void webtv1_base_map(address_map& map);
	void webtv1_retail_map(address_map& map);
	void webtv1_bfe_map(address_map& map);
};

//
// WebTV stores the AppROM across 2 flash chips. The AppROM firmware is upgradable over a network.
//
// There's two 16-bit flash chips striped across a 32-bit bus. The chip with the upper 16-bits is labeled U0501, and lower is U0502.
//
// WebTV supports flash configurations to allow 1MB (2 x 4Mbit chips), 2MB (2 x 8Mbit chips) and 4MB (2 x 16Mbit chips) AppROM images.
// The 1MB flash configuration was likely used very early in development, although this hasn't been confirmed. Production boxes used
// the 2MB configuration, while debug units and the Japan trial units likely used the 4MB configuration (although this hasn't been
// confirmed).
//
// 1MB:            AM29F400AT + AM29F400AT (NOT CONFIRMED)
// 2MB Production: AM29F800BT + AM29F800BT
// 4MB debug/JP:   MX29F1610  + MX29F1610  (NOT CONFIRMED)
//
// WebTV supported SO-44 flash chips:
//
// Fujitsu:
//    MBM29F400B:  bottom bs, 5v 4Mbit  device id=0x22ab
//    MBM29F400T:  top bs,    5v 4Mbit  device id=0x2223 (supported)
//    MBM29F800B:  bottom bs, 5v 8Mbit  device id=0x2258
//    MBM29F800T:  top bs,    5v 8Mbit  device id=0x22d6
//
// AMD:
//    AM29F400AB:  bottom bs, 5v 4Mbit  device id=0x22ab
//    AM29F400AT:  top bs,    5v 4Mbit  device id=0x2223
//    AM29F800BB:  bottom bs, 5v 8Mbit  device id=0x2258 (supported)
//    AM29F800BT:  top bs,    5v 8Mbit  device id=0x22d6
//
// Macronix:
//    MX29F1610:   top bs,    5v 16Mbit device id=0x00f1
//
// bank0_flash0 = U0501
// bank0_flash1 = U0502
//
// NOTE: If you dump these chips from a real box you may need to byte swap the output, as the bus is big-endian.
//
void webtv1_state::bank0_flash_w(offs_t offset, uint32_t data)
{
	logerror("%s: bank0_flash_w 0x1f%06x = %08x\n", machine().describe_context(), offset, data);
	uint16_t upper_value = (data >> 16) & 0xffff;
	m_flash0->write(offset, upper_value);
	uint16_t lower_value = data & 0xffff;
	m_flash1->write(offset, lower_value);
}

uint32_t webtv1_state::bank0_flash_r(offs_t offset)
{
	//logerror("%s: bank0_flash_r 0x1f%06x\n", machine().describe_context(), offset);
	uint16_t upper_value = m_flash0->read(offset);
	uint16_t lower_value = m_flash1->read(offset);
	return (upper_value << 16) | (lower_value);
}

// WebTV's firmware writes the flashing code to the lower 256 bytes of RAM
// The flash ID instructions are written first, then the flash erase instructions then the flash program instructions.
// Since everything is written to the same place, the drc cache becomes out of sync and just re-executes the ID instructions.
// This allows us to capture when new code is written and then clear the drc cache.
uint8_t webtv1_state::ram_flasher_r(offs_t offset)
{
	return ram_flasher[offset & (RAM_FLASHER_SIZE - 1)];
}
void webtv1_state::ram_flasher_w(offs_t offset, uint8_t data)
{
	if (offset == 0)
		// New code is being written, clear drc cache.
		m_maincpu->set_drc_cache_dirty_bit();

	ram_flasher[offset & (RAM_FLASHER_SIZE - 1)] = data;
}

void webtv1_state::webtv1_base_map(address_map &map)
{
	map.global_mask(0x1fffffff);

	// RAM
	map(0x00000000, 0x001fffff).ram().mirror(0x600000).share("ram"); // 2MB configuration (retail)
	//map(0x00000000, 0x003fffff).ram().mirror(0x400000).share("ram"); // 4MB configuration (dev/debug)
	//map(0x00000000, 0x007fffff).ram().share("ram");                  // 8MB configuration (full)
	// The RAM flash code gets mirrored across the entire RAM region.
	map(0x00000000, 0x00000000 + (RAM_FLASHER_SIZE - 1)).rw(FUNC(webtv1_state::ram_flasher_r), FUNC(webtv1_state::ram_flasher_w));
	map(0x00200000, 0x00200000 + (RAM_FLASHER_SIZE - 1)).rw(FUNC(webtv1_state::ram_flasher_r), FUNC(webtv1_state::ram_flasher_w));
	map(0x00400000, 0x00400000 + (RAM_FLASHER_SIZE - 1)).rw(FUNC(webtv1_state::ram_flasher_r), FUNC(webtv1_state::ram_flasher_w));
	map(0x00600000, 0x00600000 + (RAM_FLASHER_SIZE - 1)).rw(FUNC(webtv1_state::ram_flasher_r), FUNC(webtv1_state::ram_flasher_w));

	// SPOT
	map(0x04000000, 0x04000fff).m(m_spotasic, FUNC(spot_asic_device::bus_unit_map));
	map(0x04001000, 0x04001fff).m(m_spotasic, FUNC(spot_asic_device::rom_unit_map));
	map(0x04002000, 0x04002fff).m(m_spotasic, FUNC(spot_asic_device::aud_unit_map));
	map(0x04003000, 0x04003fff).m(m_spotasic, FUNC(spot_asic_device::vid_unit_map));
	map(0x04004000, 0x04004fff).m(m_spotasic, FUNC(spot_asic_device::dev_unit_map));
	map(0x04005000, 0x04005fff).m(m_spotasic, FUNC(spot_asic_device::mem_unit_map));
}

void webtv1_state::webtv1_retail_map(address_map &map)
{
	webtv1_base_map(map);

	map(0x1f000000, 0x1f3fffff).rw(FUNC(webtv1_state::bank0_flash_r), FUNC(webtv1_state::bank0_flash_w)).share("bank0");
	map(0x1f800000, 0x1fffffff).rom().region("bank1", 0);
}

void webtv1_state::webtv1_bfe_map(address_map &map)
{
	webtv1_base_map(map);

	// bank1 is not populated on bfe-type units
	map(0x1f800000, 0x1fdfffff).rom().region("bank0_mask", 0); // it's not clear whether bank0 was all flash or all mask on bfe-type units
	map(0x1fe00000, 0x1fffffff).rw(FUNC(webtv1_state::bank0_flash_r), FUNC(webtv1_state::bank0_flash_w)).share("bank0_flash");
}

void webtv1_state::webtv1_base(machine_config &config)
{
	config.set_default_layout(layout_webtv);

	R4640BE(config, m_maincpu, SYSCLOCK*2);
	m_maincpu->set_icache_size(0x2000);
	m_maincpu->set_dcache_size(0x2000);

	DS2401(config, m_serial_id, 0);

	I2C_24C01(config, m_nvram, 0);

	SPOT_ASIC(config, m_spotasic, SYSCLOCK);
	m_spotasic->set_hostcpu(m_maincpu);
	m_spotasic->set_serial_id(m_serial_id);
	m_spotasic->set_nvram(m_nvram);
}

void webtv1_state::webtv1_sony(machine_config& config)
{
	webtv1_base(config);
	m_manufacturer_code = 0x00; // Sony
	AMD_29F800B_16BIT(config, m_flash0, 0);
	AMD_29F800B_16BIT(config, m_flash1, 0);
	m_maincpu->set_addrmap(AS_PROGRAM, &webtv1_state::webtv1_retail_map);
}

void webtv1_state::webtv1_philips(machine_config& config)
{
	webtv1_base(config);
	m_manufacturer_code = 0x10; // Philips-Magnavox
	AMD_29F800B_16BIT(config, m_flash0, 0);
	AMD_29F800B_16BIT(config, m_flash1, 0);
	m_maincpu->set_addrmap(AS_PROGRAM, &webtv1_state::webtv1_retail_map);
}

void webtv1_state::webtv1_bfe(machine_config& config)
{
	webtv1_base(config);
	m_manufacturer_code = 0x20; // WebTV Generic
	AMD_29F800B_16BIT(config, m_flash0, 0);
	AMD_29F800B_16BIT(config, m_flash1, 0);
	m_maincpu->set_addrmap(AS_PROGRAM, &webtv1_state::webtv1_bfe_map);
}

//void webtv1_state::webtv1_japan(machine_config& config)
//{
//	webtv1_base(config);
//	m_manufacturer_code = 0x00; // Sony
//	MACRONIX_29F1610(config, m_flash0, 0);
//	MACRONIX_29F1610(config, m_flash1, 0);
//	m_maincpu->set_addrmap(AS_PROGRAM, &webtv1_state::webtv1_retail_map);
//}

void webtv1_state::machine_start()
{

}

void webtv1_state::machine_reset()
{

}

// Sysconfig options are usually configured via resistors on the board.
static INPUT_PORTS_START( retail_sys_config )
	PORT_START("sys_config")

	PORT_DIPUNUSED_DIPLOC(0x01, 0x01, "SW1:1")
	PORT_DIPUNUSED_DIPLOC(0x02, 0x02, "SW1:2")

	PORT_DIPNAME(0x0c, 0x0c, "Board type")
	PORT_DIPLOCATION("SW1:3,4")
	PORT_DIPSETTING(0x00, "Reserved")
	PORT_DIPSETTING(0x04, "Reserved")
	PORT_DIPSETTING(0x08, "Trial-type board")
	PORT_DIPSETTING(0x0c, "FCS board (retail)")

	// might influence behavior on certain firmware versions? everything beyond Rev 7 is not confirmed to actually exist
	PORT_DIPNAME(0xf0, 0x80, "Board revision")
	PORT_DIPLOCATION("SW1:5,6,7,8")
	PORT_DIPSETTING(0x00, "Rev 15")
	PORT_DIPSETTING(0x10, "Rev 14")
	PORT_DIPSETTING(0x20, "Rev 13")
	PORT_DIPSETTING(0x30, "Rev 12")
	PORT_DIPSETTING(0x40, "Rev 11")
	PORT_DIPSETTING(0x50, "Rev 10")
	PORT_DIPSETTING(0x60, "Rev 9")
	PORT_DIPSETTING(0x70, "Rev 8")
	PORT_DIPSETTING(0x80, "Rev 7")
	PORT_DIPSETTING(0x90, "Rev 6")
	PORT_DIPSETTING(0xa0, "Rev 5")
	PORT_DIPSETTING(0xb0, "Rev 4")
	PORT_DIPSETTING(0xc0, "Rev 3")
	PORT_DIPSETTING(0xd0, "Rev 2")
	PORT_DIPSETTING(0xe0, "Rev 1")
	PORT_DIPSETTING(0xf0, "Rev 0")

	PORT_DIPUNUSED_DIPLOC(0x100, 0x100, "SW1:9")
	PORT_DIPUNUSED_DIPLOC(0x200, 0x200, "SW1:10")
	PORT_DIPUNUSED_DIPLOC(0x400, 0x400, "SW1:11")

	// A 24.54MHz or 29.5MHz crystal drives the SAA7187 encoder chip. These pixel clocks are divisors of that input.
	PORT_DIPNAME(0x800, 0x800, "NTSC/PAL")
	PORT_DIPLOCATION("SW1:12")
	PORT_DIPSETTING(0x000, "PAL mode w/ 14.75MHz pixel clock")
	PORT_DIPSETTING(0x800, "NTSC mode w/ 12.26MHz pixel clock")

	PORT_DIPUNUSED_DIPLOC(0x1000, 0x1000, "SW1:13")

	PORT_DIPNAME(0x2000, 0x2000, "CPU output buffers")
	PORT_DIPLOCATION("SW1:14")
	PORT_DIPSETTING(0x0000, "83% CPU output buffers on reset")
	PORT_DIPSETTING(0x2000, "50% CPU output buffers on reset")

	PORT_DIPNAME(0xc000, 0x8000, "CPU clock multiplier")
	PORT_DIPLOCATION("SW1:15,16")
	PORT_DIPSETTING(0x0000, "CPU clock = 5X bus clock")
	PORT_DIPSETTING(0x4000, "CPU clock = 4X bus clock")
	PORT_DIPSETTING(0x8000, "CPU clock = 2X bus clock")
	PORT_DIPSETTING(0xc000, "CPU clock = 3X bus clock")

	PORT_DIPNAME(0x10000, 0x00000, "vidUnit clock source")
	PORT_DIPLOCATION("SW1:17")
	PORT_DIPSETTING(0x00000, "Use external video clock")
	PORT_DIPSETTING(0x10000, "SPOT controlled video clock")

	PORT_DIPNAME(0x20000, 0x00000, "audUnit clock source")
	PORT_DIPLOCATION("SW1:18")
	PORT_DIPSETTING(0x00000, "SPOT controlled DAC clock")
	PORT_DIPSETTING(0x20000, "Use external DAC clock")

	PORT_DIPNAME(0xc0000, 0xc0000, "audUnit DAC type")
	PORT_DIPLOCATION("SW1:19,20")
	PORT_DIPSETTING(0x00000, "Reserved")
	PORT_DIPSETTING(0x40000, "Reserved")
	PORT_DIPSETTING(0x80000, "Reserved")
	PORT_DIPSETTING(0xc0000, "AKM 4310/4309")

	PORT_DIPNAME(0x300000, 0x200000, "Memory vendor")
	PORT_DIPLOCATION("SW1:21,22")
	PORT_DIPSETTING(0x000000, "Other")
	PORT_DIPSETTING(0x100000, "Samsung")
	PORT_DIPSETTING(0x200000, "Fujitsu")
	PORT_DIPSETTING(0x300000, "NEC")

	PORT_DIPNAME(0xc00000, 0xC00000, "Memory speed")
	PORT_DIPLOCATION("SW1:23,24")
	PORT_DIPSETTING(0x000000, "100MHz parts")
	PORT_DIPSETTING(0x400000, "66MHz parts")
	PORT_DIPSETTING(0x800000, "77MHz parts")
	PORT_DIPSETTING(0xc00000, "83MHz parts")

	PORT_DIPNAME(0x3000000, 0x3000000, "Bank 1 ROM speed")
	PORT_DIPLOCATION("SW1:25,26")
	PORT_DIPSETTING(0x0000000, "200ns/100ns")
	PORT_DIPSETTING(0x1000000, "100ns/50ns")
	PORT_DIPSETTING(0x2000000, "90ns/45ns")
	PORT_DIPSETTING(0x3000000, "120ns/60ns")

	PORT_DIPNAME(0x4000000, 0x0000000, "Bank 1 ROM mode")
	PORT_DIPLOCATION("SW1:27")
	PORT_DIPSETTING(0x0000000, "No page mode")
	PORT_DIPSETTING(0x4000000, "Supports page mode")

	PORT_DIPNAME(0x8000000, 0x8000000, "Bank 1 ROM type")
	PORT_DIPLOCATION("SW1:28")
	PORT_DIPSETTING(0x0000000, "Flash ROM")
	PORT_DIPSETTING(0x8000000, "Mask ROM")

	PORT_DIPNAME(0x30000000, 0x20000000, "Bank 0 ROM speed")
	PORT_DIPLOCATION("SW1:29,30")
	PORT_DIPSETTING(0x00000000, "200ns/100ns")
	PORT_DIPSETTING(0x10000000, "100ns/50ns")
	PORT_DIPSETTING(0x20000000, "90ns/45ns")
	PORT_DIPSETTING(0x30000000, "120ns/60ns")

	PORT_DIPNAME(0x40000000, 0x00000000, "Bank 0 ROM mode")
	PORT_DIPLOCATION("SW1:31")
	PORT_DIPSETTING(0x00000000, "No page mode")
	PORT_DIPSETTING(0x40000000, "Supports page mode")

	PORT_DIPNAME(0x80000000, 0x00000000, "Bank 0 ROM type")
	PORT_DIPLOCATION("SW1:32")
	PORT_DIPSETTING(0x00000000, "Flash ROM")
	PORT_DIPSETTING(0x80000000, "Mask ROM")
INPUT_PORTS_END

// This is emulator-specific config options that go beyond sysconfig offers.
// TODO: Clean this up when we move smartcard bangserial to its own serial device
static INPUT_PORTS_START( emu_config )
	PORT_START("emu_config")

	PORT_CONFNAME(0x03, 0x00, "Pixel buffer index")
	PORT_CONFSETTING(0x00, "Use pixel buffer 0 (standard output)")
	PORT_CONFSETTING(0x01, "Use pixel buffer 1")

	PORT_CONFNAME(0x0c, 0x0c, "Smartcard bangserial")
	PORT_CONFSETTING(0x00, "Off")
	PORT_CONFSETTING(0x04, "V1 bangserial data")
	PORT_CONFSETTING(0x08, "V2 bangserial data")
	PORT_CONFSETTING(0x0c, "Autodetect")
INPUT_PORTS_END

static INPUT_PORTS_START( retail_input )
	PORT_INCLUDE(retail_sys_config)
	PORT_INCLUDE(emu_config)
INPUT_PORTS_END

ROM_START( wtv1sony )
	ROM_REGION(0x8, "serial_id", 0)     /* Electronic Serial DS2401 */
	ROM_LOAD("ds2401.bin", 0x0000, 0x0008, NO_DUMP)

	ROM_REGION32_BE(0x800000, "bank1", 0)
	ROM_LOAD("bootrom.o", 0x000000, 0x200000, CRC(5ad8f7b6) SHA1(a5c411f5f0126e79a0d925822062203c2272faf8)) /* bf0boot 105 pre-decoded; from archival efforts of the WebTV update servers */
	ROM_RELOAD(0x200000, 0x200000)
	ROM_RELOAD(0x400000, 0x200000)
	ROM_RELOAD(0x600000, 0x200000)
ROM_END

ROM_START( wtv1phil )
	ROM_REGION(0x8, "serial_id", 0)     /* Electronic Serial DS2401 */
	ROM_LOAD("ds2401.bin", 0x0000, 0x0008, NO_DUMP)

	ROM_REGION32_BE(0x800000, "bank1", 0)
	ROM_LOAD("bootrom.o", 0x000000, 0x200000, CRC(5ad8f7b6) SHA1(a5c411f5f0126e79a0d925822062203c2272faf8)) /* bf0boot 105 pre-decoded; from archival efforts of the WebTV update servers */
	ROM_RELOAD(0x200000, 0x200000)
	ROM_RELOAD(0x400000, 0x200000)
	ROM_RELOAD(0x600000, 0x200000)
ROM_END

//ROM_START( wtv1jpn )
//	ROM_REGION(0x8, "serial_id", 0)     /* Electronic Serial DS2401 */
//	ROM_LOAD("ds2401.bin", 0x0000, 0x0008, NO_DUMP)
//
//	ROM_REGION32_BE(0x800000, "bank1", 0)
//	ROM_LOAD("bootrom.o", 0x000000, 0x400000, NO_DUMP) /* This ROM is expected to be a japan-4MegBoot type */
//	ROM_RELOAD(0x400000, 0x400000)
//ROM_END

ROM_START( wtv1bfe )
	ROM_REGION(0x8, "serial_id", 0)     /* Electronic Serial DS2401 */
	ROM_LOAD("ds2401.bin", 0x0000, 0x0008, NO_DUMP)

	ROM_REGION32_BE(0x600000, "bank0_mask", 0)
	ROM_SYSTEM_BIOS(0, "bfe105", "Standard bfe-type BootROM (1.0, build 105)")
	ROMX_LOAD("bootrom.o", 0x400000, 0x200000, CRC(71c321db) SHA1(6a39e064fb2312d70728b8105de990762226bd07), ROM_BIOS(0))
	ROM_SYSTEM_BIOS(1, "prealpha-boot", "Pre-alpha bfe-type BootROM (05/17/96?)") // this BootROM was built from source - the exact version number and checksum is unknown at this time
	ROMX_LOAD("prealpha-boot.o", 0x400000, 0x200000, NO_DUMP, ROM_BIOS(1))
ROM_END

//    YEAR   NAME      PARENT    COMPAT  MACHINE         INPUT         CLASS         INIT        COMPANY                FULLNAME                                    FLAGS
CONS( 1996,  wtv1sony,        0,      0, webtv1_sony,    retail_input, webtv1_state, empty_init, "Sony",                "INT-W100 WebTV Internet Terminal",         MACHINE_NOT_WORKING | MACHINE_IMPERFECT_TIMING )
CONS( 1996,  wtv1phil,        0,      0, webtv1_philips, retail_input, webtv1_state, empty_init, "Philips-Magnavox",    "MAT960 WebTV Internet Terminal",           MACHINE_NOT_WORKING | MACHINE_IMPERFECT_TIMING )
CONS( 1996,  wtv1bfe,         0,      0, webtv1_bfe,     retail_input, webtv1_state, empty_init, "WebTV Networks Inc.", "prototype/trial WebTV FCS box (bfe-type)", MACHINE_NOT_WORKING | MACHINE_IS_INCOMPLETE | MACHINE_IMPERFECT_TIMING )
//CONS( 1996?, wtv1jpn,         0,      0, webtv1_japan,   retail_input, webtv1_state, empty_init, "Sony?",               "unknown Japanese WebTV trial box",         MACHINE_NOT_WORKING | MACHINE_IMPERFECT_TIMING )
