// license:BSD-3-Clause
// copyright-holders:FairPlay137

/***********************************************************************************************

    solo1_asic.cpp

    WebTV Networks Inc. SOLO1 ASIC

    This ASIC controls most of the I/O on the 2nd generation WebTV hardware.

    This implementation is based off of both the archived technical specifications, as well as
    the various reverse-engineering efforts of the WebTV community.

    The technical specifications that this implementation is based on can be found here:
    http://wiki.webtv.zone/misc/SOLO1/SOLO1_ASIC_Spec.pdf

************************************************************************************************/

#ifndef MAME_MACHINE_SOLO1_ASIC_H
#define MAME_MACHINE_SOLO1_ASIC_H

#pragma once

#include "diserial.h"

#include "cpu/mips/mips3.h"
#include "machine/ds2401.h"
#include "machine/i2cmem.h"
#include "machine/watchdog.h"
#include "sound/dac.h"
#include "speaker.h"

class solo1_asic_device : public device_t, public device_serial_interface, public device_video_interface
{
public:
	// construction/destruction
	solo1_asic_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
    
    void bus_unit_map(address_map &map);
    void rio_unit_map(address_map &map);
    void aud_unit_map(address_map &map);
    void vid_unit_map(address_map &map);
    void dev_unit_map(address_map &map);
    void mem_unit_map(address_map &map);
    void gfx_unit_map(address_map &map);
    void dve_unit_map(address_map &map);
    void div_unit_map(address_map &map);
    void pot_unit_map(address_map &map);
    void suc_unit_map(address_map &map);
    void mod_unit_map(address_map &map);

	template <typename T> void set_hostcpu(T &&tag) { m_hostcpu.set_tag(std::forward<T>(tag)); }
	template <typename T> void set_serial_id(T &&tag) { m_serial_id.set_tag(std::forward<T>(tag)); }
	template <typename T> void set_nvram(T &&tag) { m_nvram.set_tag(std::forward<T>(tag)); }

	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

    void irq_aud_w(uint32_t value);
    void irq_vid_w(uint32_t value);
    void irq_rio_w(uint32_t value);

	void set_pot_irq(uint8_t mask, int state);

    uint32_t reg_bus_r(offs_t offset);
    void reg_bus_w(offs_t offset, uint32_t data);
    
    //uint32_t reg_rio_r(offs_t offset);
    //void reg_rio_w(offs_t offset, uint32_t data);

    uint32_t reg_dev_irold_r(offs_t offset);
    void reg_dev_irold_w(offs_t offset, uint32_t data);

    uint32_t reg_dev_r(offs_t offset);
    void reg_dev_w(offs_t offset, uint32_t data);
    
    uint32_t reg_mem_r(offs_t offset);
    void reg_mem_w(offs_t offset, uint32_t data);
    
    //uint32_t reg_suc_r(offs_t offset);
    //void reg_suc_w(offs_t offset, uint32_t data);
    
protected:
	// device-level overrides
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;

    uint32_t m_bus_chip_id; // SOLO chip ID
    uint32_t m_bus_chip_cntl;
    
    uint32_t m_bus_int_status;
    uint32_t m_bus_int_enable;

    uint32_t m_bus_err_status;
    uint32_t m_bus_err_enable;
    uint32_t m_bus_err_address;
    
    uint32_t m_bus_wd_reset_val;

    uint32_t m_bus_lomem_rdprot_addr;
    uint32_t m_bus_lomem_rdprot_mask;
    uint32_t m_bus_lomem_wrprot_addr;
    uint32_t m_bus_lomem_wrprot_mask;
    
    uint32_t m_bus_tmr_count;
    uint32_t m_bus_tmr_compare;

    uint32_t m_bus_gpio_int_status; // GPIO interrupt status
    uint32_t m_bus_gpio_int_enable; // GPIO interrupt enable
    uint32_t m_bus_gpio_int_polling; // GPIO interrupt polling
    
    uint32_t m_bus_aud_int_status; // audUnit interrupt status
    uint32_t m_bus_aud_int_enable; // audUnit interrupt enable
    
    uint32_t m_bus_dev_int_status; // devUnit interrupt status
    uint32_t m_bus_dev_int_enable; // devUnit interrupt enable
    
    uint32_t m_bus_vid_int_status; // vidUnit interrupt status
    uint32_t m_bus_vid_int_enable; // vidUnit interrupt enable
    
    uint32_t m_bus_rio_int_status; // RIO bus interrupt status
    uint32_t m_bus_rio_int_enable; // RIO bus interrupt enable
    uint32_t m_bus_rio_int_polling; // RIO bus interrupt polling
    
    uint32_t m_bus_tim_int_status; // Timing interrupt status
    uint32_t m_bus_tim_int_enable; // Timing interrupt enable
    
    uint32_t m_bus_reset_cause;
    
    uint32_t m_bus_java1_fence_addr_l;
    uint32_t m_bus_java1_fence_addr_h;
    uint32_t m_bus_java2_fence_addr_l;
    uint32_t m_bus_java2_fence_addr_h;

    uint32_t m_bus_memsize; // Any attempted RAM accesses above this address will trigger a bus error
    uint32_t m_bus_fence_cntl;

    uint32_t m_bus_bootmode;
    uint32_t m_bus_use_bootmode;

	uint32_t m_vid_cstart;
	uint32_t m_vid_csize;
	uint32_t m_vid_ccnt;
    uint32_t m_vid_nstart;
	uint32_t m_vid_nsize;
	uint32_t m_vid_dmacntl;

	uint32_t m_pot_hstart;
	uint32_t m_pot_hsize;
	uint32_t m_pot_vstart;
	uint32_t m_pot_vsize;
	uint16_t m_pot_cntl;
	uint32_t m_pot_blank_color;
	uint32_t m_pot_cline;
	uint32_t m_pot_hintline;
	uint32_t m_pot_intenable;
	uint32_t m_pot_intstat;

	uint32_t m_pot_drawstart;
	uint32_t m_pot_drawvsize;

private:
    required_device<mips3_device> m_hostcpu;
	required_device<ds2401_device> m_serial_id;
	required_device<i2cmem_device> m_nvram;
    
	required_device<screen_device> m_screen;
    
	required_device_array<dac_word_interface, 2> m_dac;
	required_device<speaker_device> m_lspeaker;
	required_device<speaker_device> m_rspeaker;
    
	required_device<watchdog_timer_device> m_watchdog;


	output_finder<1> m_power_led;
	output_finder<1> m_connect_led;
	output_finder<1> m_message_led;

	void vblank_irq(int state);
    
    uint32_t m_compare_armed;
    
    bool m_aud_dma_ongoing;

    void solo1_update_cycle_counting();

    emu_timer *m_sys_timer = nullptr;
    TIMER_CALLBACK_MEMBER(sys_timer_callback);

	emu_timer *dac_update_timer = nullptr;
	TIMER_CALLBACK_MEMBER(dac_update);

    uint32_t m_dev_irold;
    uint32_t m_dev_led;
    uint32_t m_dev_id_chip_cntl;
    uint32_t m_dev_iic_cntl;

    uint32_t m_dev_gpio_in;
    uint32_t m_dev_gpio_out;
    uint32_t m_dev_gpio_en;

    uint32_t m_dev_ir_in_sample_int;
    uint32_t m_dev_ir_in_reject_int;
    uint32_t m_dev_ir_in_transition_data;
    uint32_t m_dev_ir_in_status_cntl;
    
    uint32_t m_dev_ir_out_fifo;
    uint32_t m_dev_ir_out_status;
    uint32_t m_dev_ir_out_period;
    uint32_t m_dev_ir_out_on;
    uint32_t m_dev_ir_out_current_period;
    uint32_t m_dev_ir_out_current_on;
    uint32_t m_dev_ir_out_current_count;

    uint32_t m_dev_parallel_data;
    uint32_t m_dev_parallel_ctrl;
    uint32_t m_dev_parallel_status;
    uint32_t m_dev_parallel_cnfg;
    uint32_t m_dev_parallel_fifo_ctrl;
    uint32_t m_dev_parallel_fifo_status;
    uint32_t m_dev_parallel_timeout;
    uint32_t m_dev_parallel_stat2;
    uint32_t m_dev_parallel_int_enable;
    uint32_t m_dev_parallel_int_status;
    uint32_t m_dev_parallel_clr_int;
    uint32_t m_dev_parallel_enable;
    
    uint32_t m_dev_diag;
    uint32_t m_dev_devdiag;

    uint32_t m_mem_timing; // SDRAM timing parameters
    uint32_t m_mem_cntl;
    uint32_t m_mem_burp; // Memory access arbitration control
    uint32_t m_mem_refresh_cntl; // SDRAM refresh control
    uint32_t m_mem_cmd; // SDRAM commands

    uint32_t m_sucgpu_tff_hr; // TX FIFO data
    uint32_t m_sucgpu_tff_hrsrw; // TX FIFO data (debug)
    uint32_t m_sucgpu_tff_trg; // TX FIFO trigger level
    uint32_t m_sucgpu_tff_cnt; // Current number of entries in TX FIFO
    uint32_t m_sucgpu_tff_max; // Maximum TX FIFO depth
    uint32_t m_sucgpu_tff_ctl;
    uint32_t m_sucgpu_tff_sta;
    uint32_t m_sucgpu_tff_gcr;

    /* busUnit registers */

    uint32_t reg_0000_r();          // BUS_CHIPID (read-only)
    uint32_t reg_0004_r();          // BUS_CHPCNTL (read)
    void reg_0004_w(uint32_t data); // BUS_CHPCNTL (write)
    
};

DECLARE_DEVICE_TYPE(SOLO1_ASIC, solo1_asic_device)

#endif // MAME_MACHINE_SOLO1_ASIC_H
