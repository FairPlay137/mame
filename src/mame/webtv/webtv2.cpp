// license:BSD-3-Clause
// copyright-holders:FairPlay137

/***************************************************************************************
 *
 * WebTV LC2 (1997)
 * 
 * The WebTV line of products was a set of thin clients meant to make the emerging-at-
 * -the-time Internet accessible to those who weren't as familiar with computers,
 * adapting websites for display on television sets, simplifying the control scheme to
 * work with just a remote control and an optional keyboard, and optimizing them to work
 * smoothly on low-cost hardware. Later on in its life, it was rebranded as MSN TV.
 * 
 * LC2, shorthand for "Low Cost v2", was the second generation of the WebTV hardware.
 * It added graphics acceleration, an on-board printer port, and the ability to use a
 * hard drive, a TV tuner, and satellite receiver circuitry. It was also faster.
 * It uses a custom ASIC designed by WebTV Networks Inc. known as SOLO.
 *
 * The original LC2 boards used a MIPS IDT R4640 clocked at 167MHz, although later
 * board revisions switched to a MIPS RM5230, although some LC2 dev boards and trial
 * run boards had RM5230s back when the retail models still used the R4640.
 * 
 * This driver would not have been possible without the efforts of the WebTV community
 * to preserve technical specifications, as well as the various reverse-engineering
 * efforts that were made.
 * 
 * The technical specifications that this implementation is based on can be found here:
 * http://wiki.webtv.zone/misc/SOLO1/SOLO1_ASIC_Spec.pdf
 * 
 * Stuff that still needs to be done:
 * - Different configurations for disk units and flash units (since there is one known
 *   surviving European trial unit in a flash-only configuration)
 * - Dummy emulation of tuner (required for booting Plus ROMs all the way)
 * - SOLO gfxUnit-based graphics acceleration
 * - Support for the MIPS RM5230 configuration
 * 
 * This code is currently undergoing major refactors. Stuff may not work as expected,
 * but it'll be more in line with the current standards.
 * 
 ***************************************************************************************/

#include "emu.h"

#include "cpu/mips/mips3.h"
#include "solo1_asic.h"
#include "bus/ata/ataintf.h"
#include "bus/ata/hdd.h"

#include "webtv.lh"

#include "main.h"
#include "screen.h"

#define SYSCLOCK 83300000 // NOT CONFIRMED!

namespace {

class webtv2_state : public driver_device
{
public:
	// constructor
	webtv2_state(const machine_config& mconfig, device_type type, const char* tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_soloasic(*this, "solo"),
		m_serial_id(*this, "serial_id"),
		m_nvram(*this, "nvram")
	{ }

	void webtv2_base(machine_config& config);
	void webtv2_sony(machine_config& config);
	void webtv2_philips(machine_config& config);

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;

private:
	required_device<mips3_device> m_maincpu;
	required_device<solo1_asic_device> m_soloasic;
	
	required_device<ds2401_device> m_serial_id;
	required_device<i2cmem_device> m_nvram;

	void webtv2_map(address_map& map);
};

void webtv2_state::webtv2_map(address_map &map)
{
	map.global_mask(0x1fffffff);

	// RAM
	map(0x00000000, 0x03ffffff).ram().share("mainram"); // TODO: allocating all 64MB is inaccurate to retail hardware!

	// SOLO
	map(0x04000000, 0x04000fff).m(m_soloasic, FUNC(solo1_asic_device::bus_unit_map));
	map(0x04001000, 0x04001fff).m(m_soloasic, FUNC(solo1_asic_device::rio_unit_map));
	map(0x04002000, 0x04002fff).m(m_soloasic, FUNC(solo1_asic_device::aud_unit_map));
	map(0x04003000, 0x04003fff).m(m_soloasic, FUNC(solo1_asic_device::vid_unit_map));
	map(0x04004000, 0x04004fff).m(m_soloasic, FUNC(solo1_asic_device::dev_unit_map));
	map(0x04005000, 0x04005fff).m(m_soloasic, FUNC(solo1_asic_device::mem_unit_map));
	map(0x04006000, 0x04006fff).m(m_soloasic, FUNC(solo1_asic_device::gfx_unit_map));
	map(0x04007000, 0x04007fff).m(m_soloasic, FUNC(solo1_asic_device::dve_unit_map));
	map(0x04008000, 0x04008fff).m(m_soloasic, FUNC(solo1_asic_device::div_unit_map));
	map(0x04009000, 0x04009fff).m(m_soloasic, FUNC(solo1_asic_device::pot_unit_map));
	map(0x0400a000, 0x0400afff).m(m_soloasic, FUNC(solo1_asic_device::suc_unit_map));
	map(0x0400b000, 0x0400bfff).m(m_soloasic, FUNC(solo1_asic_device::mod_unit_map));

	// expansion device areas
	//map(0x04800000, 0x04ffffff).ram().share("exp1");
	//map(0x05000000, 0x057fffff).ram().share("exp2");
	//map(0x05800000, 0x05ffffff).ram().share("exp3");
	//map(0x06000000, 0x067fffff).ram().share("exp4");
	//map(0x06800000, 0x06ffffff).ram().share("exp5");
	//map(0x07000000, 0x077fffff).ram().share("exp6");
	//map(0x07800000, 0x07ffffff).ram().share("exp7");
	
	// 0x1d000000 - 0x1d3fffff: secondary device 4 (unassigned)
	// 0x1d400000 - 0x1d7fffff: secondary device 5 (IDE CD-ROM)
	// 0x1d800000 - 0x1dbfffff: secondary device 6 (IDE CD-ROM)
	// 0x1dc00000 - 0x1dffffff: secondary device 7 (unassigned)

	// 0x1e000000 - 0x1e3fffff: primary device 0 (modem/ethernet)
	// 0x1e400000 - 0x1e7fffff: primary device 1 (IDE hard disk)
	// 0x1e800000 - 0x1ebfffff: primary device 2 (IDE hard disk)
	// 0x1ec00000 - 0x1effffff: primary device 3 (unassigned)

	map(0x1f000000, 0x1f7fffff).rom().region("bank0", 0); // Flash ROM
	map(0x1f800000, 0x1fffffff).rom().region("bank1", 0); // Mask ROM
}

void webtv2_state::webtv2_base(machine_config& config)
{
	config.set_default_layout(layout_webtv);

	R4640BE(config, m_maincpu, SYSCLOCK * 2); // TODO: implement RM5230 emulation
	m_maincpu->set_icache_size(0x2000);
	m_maincpu->set_dcache_size(0x2000);
	m_maincpu->set_addrmap(AS_PROGRAM, &webtv2_state::webtv2_map);

	DS2401(config, m_serial_id, 0);

	I2C_24C01(config, m_nvram, 0);
	m_nvram->set_e0(0);
	m_nvram->set_wc(1);

	SOLO1_ASIC(config, m_soloasic, SYSCLOCK);
	m_soloasic->set_hostcpu(m_maincpu);
}

void webtv2_state::webtv2_sony(machine_config& config)
{
	webtv2_base(config);
	// TODO: differentiate manufacturers via 
}

void webtv2_state::webtv2_philips(machine_config& config)
{
	webtv2_base(config);
	// TODO: differentiate manufacturers via emulated serial id
}

void webtv2_state::machine_start()
{

}

void webtv2_state::machine_reset()
{

}

ROM_START( wtv2sony )
	ROM_REGION(0x8, "serial_id", 0)     /* Electronic Serial DS2401 */
	ROM_LOAD("ds2401.bin", 0x0000, 0x0008, NO_DUMP)

	ROM_REGION32_BE(0x800000, "bank1", 0)
	ROM_SYSTEM_BIOS(0, "2046ndbg", "Standard Boot ROM (2.0, build 2046)")
	ROMX_LOAD("2046ndbg.o", 0x000000, 0x200000, NO_DUMP, ROM_BIOS(0))
	ROM_RELOAD(0x200000, 0x200000)
	ROM_RELOAD(0x400000, 0x200000)
	ROM_RELOAD(0x600000, 0x200000)
	ROM_SYSTEM_BIOS(1, "joedebug", "Joe Britt's Debug Boot ROM (build 32767)")
	ROMX_LOAD("joedebug.o", 0x000000, 0x200000, NO_DUMP, ROM_BIOS(1))
	ROM_RELOAD(0x200000, 0x200000)
	ROM_RELOAD(0x400000, 0x200000)
	ROM_RELOAD(0x600000, 0x200000)
ROM_END

ROM_START( wtv2phil )
	ROM_REGION(0x8, "serial_id", 0)     /* Electronic Serial DS2401 */
	ROM_LOAD("ds2401.bin", 0x0000, 0x0008, NO_DUMP)

	ROM_REGION32_BE(0x800000, "bank1", 0)
	ROM_LOAD("2046ndbg.o", 0x000000, 0x200000, NO_DUMP)
	ROM_RELOAD(0x200000, 0x200000)
	ROM_RELOAD(0x400000, 0x200000)
	ROM_RELOAD(0x600000, 0x200000)
ROM_END

}

//    YEAR  NAME      PARENT  COMPAT  MACHINE        INPUT  CLASS         INIT        COMPANY               FULLNAME                        FLAGS
CONS( 1997, wtv2sony,      0,      0, webtv2_sony,       0, webtv2_state, empty_init, "Sony",               "INT-W200 WebTV Plus Receiver", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_GRAPHICS | MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_TIMING | MACHINE_NODEVICE_MICROPHONE | MACHINE_NODEVICE_PRINTER )
CONS( 1997, wtv2phil,      0,      0, webtv2_philips,    0, webtv2_state, empty_init, "Philips-Magnavox",   "MAT972 WebTV Plus Receiver",   MACHINE_NOT_WORKING | MACHINE_IMPERFECT_GRAPHICS | MACHINE_IMPERFECT_SOUND | MACHINE_IMPERFECT_TIMING | MACHINE_NODEVICE_MICROPHONE | MACHINE_NODEVICE_PRINTER )
