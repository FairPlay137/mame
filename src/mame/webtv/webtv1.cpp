/***************************************************************************************
 *
 * WebTV FCS (1996)
 * 
 * The WebTV line of products was an early attempt to bring the Internet to the
 * television. Later on in its life, it was rebranded as MSN TV.
 * 
 * FCS, shorthand for First Customer Ship, was the first generation of WebTV hardware.
 * Its ASIC, known as SPOT or FIDO, is much simpler than SOLO.
 * 
 * A typical retail configuration uses a MIPS IDT R4640 clocked at 112MHz, with 2MB of
 * on-board RAM, 2MB of flash memory (using two 16-bit 1MB AMD 29F800B chips) for the
 * updatable software, and 2MB of mask ROM.
 * 
 * This driver would not have been possible without the efforts of the WebTV community
 * to preserve technical specifications, as well as the various reverse-engineering
 * efforts that were made.
 * 
 * Current status: Breaks at bfc0c030, a little before the memory check.
 * 
 ***************************************************************************************/

#include "emu.h"

#include "bus/pc_kbd/keyboards.h"
#include "cpu/mips/mips3.h"
#include "machine/ds2401.h"
#include "machine/intelfsh.h"
#include "spot_asic.h"

#include "webtv.lh"

#include "main.h"
#include "screen.h"

#define SYSCLOCK 56000000 // TODO: confirm this is correct

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
		m_power_led(*this, "power_led"),
		m_connect_led(*this, "connect_led"),
		m_message_led(*this, "message_led")
	{ }

	void webtv1_base(machine_config& config);
	void webtv1_sony(machine_config& config);
	void webtv1_philips(machine_config& config);

protected:
	virtual void machine_start() override;
	virtual void machine_reset() override;
    
private:
	required_device<mips3_device> m_maincpu;
	required_device<spot_asic_device> m_spotasic;
	required_device<ds2401_device> m_serial_id;
	required_device<i2cmem_device> m_nvram;

	output_finder<1> m_power_led;
	output_finder<1> m_connect_led;
	output_finder<1> m_message_led;

	enum
	{
		LED_POWER = 0x4,
		LED_CONNECTED = 0x2,
		LED_MESSAGE = 0x1
	};

	void bank0_flash_w(offs_t offset, uint32_t data);
	uint32_t bank0_flash_r(offs_t offset);

	void webtv1_map(address_map& map);
};

void webtv1_state::bank0_flash_w(offs_t offset, uint32_t data)
{
	// WebTV FCS uses two AMD AM29F800BT chips on the board for storing its software.
	// One chip is for the lower 16 bits (labeled U0504), and the other is for the upper 16 bits (labeled U0503).
	// In addition, the bytes are also flipped.
	logerror("%s: bank0_flash_w 0x1f%06x = %08x\n", machine().describe_context(), offset, data);
}

uint32_t webtv1_state::bank0_flash_r(offs_t offset)
{
	logerror("%s: bank0_flash_r 0x1f%06x\n", machine().describe_context(), offset);
    return 0xFFFFFFFF;
}

void webtv1_state::webtv1_map(address_map &map)
{
	map.global_mask(0x1fffffff);

	// RAM
	map(0x00000000, 0x007fffff).ram().share("ram"); // 8MB is not accurate to retail hardware! Ideally this would be 2MB or 4MB, mirrored across this memory space

	// SPOT
	map(0x04000000, 0x04000fff).m(m_spotasic, FUNC(spot_asic_device::bus_unit_map));
	map(0x04001000, 0x04001fff).m(m_spotasic, FUNC(spot_asic_device::rom_unit_map));
	map(0x04002000, 0x04002fff).m(m_spotasic, FUNC(spot_asic_device::aud_unit_map));
	map(0x04003000, 0x04003fff).m(m_spotasic, FUNC(spot_asic_device::vid_unit_map));
	map(0x04004000, 0x04004fff).m(m_spotasic, FUNC(spot_asic_device::dev_unit_map));
	map(0x04005000, 0x04005fff).m(m_spotasic, FUNC(spot_asic_device::mem_unit_map));

	// ROM
	map(0x1f000000, 0x1f3fffff).rw(FUNC(bank0_flash_r), FUNC(webtv1_state::bank0_flash_w)).share("bank0"); // Flash ROM, 4MB (retail configuration 2MB)
	map(0x1f800000, 0x1fffffff).rom().region("bank1", 0); // Mask ROM
}

void webtv1_state::webtv1_base(machine_config &config)
{
	config.set_default_layout(layout_webtv);

	R4640BE(config, m_maincpu, SYSCLOCK*2);
	m_maincpu->set_icache_size(0x2000);
	m_maincpu->set_dcache_size(0x2000);
	m_maincpu->set_addrmap(AS_PROGRAM, &webtv1_state::webtv1_map);
	
	AMD_29F800B_16BIT(config, "bank0_flash0");
	AMD_29F800B_16BIT(config, "bank0_flash1");

	DS2401(config, m_serial_id, 0);

	I2C_24C01(config, m_nvram, 0);
	m_nvram->set_e0(0);
	m_nvram->set_wc(1);

	SPOT_ASIC(config, m_spotasic, SYSCLOCK);
	m_spotasic->set_hostcpu(m_maincpu);
	m_spotasic->set_serial_id(m_serial_id);
	m_spotasic->set_nvram(m_nvram);
}

void webtv1_state::webtv1_sony(machine_config& config)
{
	// manufacturer is determined by the contents of DS2401
	webtv1_base(config);
}

void webtv1_state::webtv1_philips(machine_config& config)
{
	// manufacturer is determined by the contents of DS2401
	webtv1_base(config);
}

void webtv1_state::machine_start()
{

}

void webtv1_state::machine_reset()
{

}

ROM_START( wtv1sony )
	ROM_REGION(0x8, "serial_id", 0)     /* Electronic Serial DS2401 */
	ROM_LOAD("ds2401.bin", 0x0000, 0x0008, CRC(46E0DE7D) SHA1(E885D37209093FAA5BFF97853BBB848D803DB011))

	ROM_REGION32_BE(0x800000, "bank1", 0)
	ROM_LOAD("bootrom.o", 0x000000, 0x200000, CRC(71C321DB) SHA1(6A39E064FB2312D70728B8105DE990762226BD07)) /* pre-decoded */
	ROM_RELOAD(0x200000, 0x200000)
	ROM_RELOAD(0x400000, 0x200000)
	ROM_RELOAD(0x600000, 0x200000)
ROM_END

ROM_START( wtv1phil )
	ROM_REGION(0x8, "serial_id", 0)     /* Electronic Serial DS2401 */
	ROM_LOAD("ds2401.bin", 0x0000, 0x0008, CRC(2E214B26) SHA1(53FCC9E7986953CC0CF149B29FC14A245EEA4E17))

	ROM_REGION32_BE(0x800000, "bank1", 0)
	ROM_LOAD("bootrom.o", 0x000000, 0x200000, CRC(71C321DB) SHA1(6A39E064FB2312D70728B8105DE990762226BD07)) /* pre-decoded */
	ROM_RELOAD(0x200000, 0x200000)
	ROM_RELOAD(0x400000, 0x200000)
	ROM_RELOAD(0x600000, 0x200000)
ROM_END

//    YEAR  NAME      PARENT  COMPAT  MACHINE        INPUT  CLASS         INIT        COMPANY               FULLNAME                            FLAGS
CONS( 1996, wtv1sony,      0,      0, webtv1_sony,       0, webtv1_state, empty_init, "Sony",               "INT-W100 WebTV Internet Terminal", MACHINE_NOT_WORKING + MACHINE_NO_SOUND )
CONS( 1996, wtv1phil,      0,      0, webtv1_philips,    0, webtv1_state, empty_init, "Philips-Magnavox",   "MAT960 WebTV Internet Terminal",   MACHINE_NOT_WORKING + MACHINE_NO_SOUND )
