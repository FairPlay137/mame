// license:BSD-3-Clause
// copyright-holders:Nicola Salmoria, Aaron Giles, Nathan Woods
/*********************************************************************

    ui/slotopt.cpp

    Internal menu for the slot options.

*********************************************************************/

#include "emu.h"

#include "ui/ui.h"
#include "ui/slotopt.h"
#include "ui/devopt.h"

#include "emuopts.h"
#include "mameopts.h"


namespace ui {
/*-------------------------------------------------
    slot_get_current_option - returns
-------------------------------------------------*/
device_slot_option *menu_slot_devices::slot_get_current_option(device_slot_interface &slot)
{
	std::string current;

	const char *slot_option_name = slot.device().tag() + 1;
	if (!slot.fixed())
	{
		current = machine().options().slot_option(slot_option_name).value();
	}
	else
	{
		if (slot.default_option() == nullptr)
			return nullptr;
		current.assign(slot.default_option());
	}

	return slot.option(current.c_str());
}

/*-------------------------------------------------
    slot_get_current_index - returns
-------------------------------------------------*/
int menu_slot_devices::slot_get_current_index(device_slot_interface &slot)
{
	const device_slot_option *current = slot_get_current_option(slot);

	if (current != nullptr)
	{
		int val = 0;
		for (auto &option : slot.option_list())
		{
			if (option.second.get() == current)
				return val;

			if (option.second->selectable())
				val++;
		}
	}

	return -1;
}

/*-------------------------------------------------
    slot_get_length - returns
-------------------------------------------------*/
int menu_slot_devices::slot_get_length(device_slot_interface &slot)
{
	int val = 0;
	for (auto &option : slot.option_list())
		if (option.second->selectable())
			val++;

	return val;
}

/*-------------------------------------------------
    slot_get_next - returns
-------------------------------------------------*/
const char *menu_slot_devices::slot_get_next(device_slot_interface &slot)
{
	int idx = slot_get_current_index(slot);
	if (idx < 0)
		idx = 0;
	else
		idx++;

	if (idx >= slot_get_length(slot))
		return "";

	return slot_get_option(slot, idx);
}

/*-------------------------------------------------
    slot_get_prev - returns
-------------------------------------------------*/
const char *menu_slot_devices::slot_get_prev(device_slot_interface &slot)
{
	int idx = slot_get_current_index(slot);
	if (idx < 0)
		idx = slot_get_length(slot) - 1;
	else
		idx--;

	if (idx < 0)
		return "";

	return slot_get_option(slot, idx);
}

/*-------------------------------------------------
    slot_get_option - returns
-------------------------------------------------*/
const char *menu_slot_devices::slot_get_option(device_slot_interface &slot, int index)
{
	if (index >= 0)
	{
		int val = 0;
		for (auto &option : slot.option_list())
		{
			if (val == index)
				return option.second->name();

			if (option.second->selectable())
				val++;
		}
	}

	return "";
}


/*-------------------------------------------------
    set_use_natural_keyboard - specifies
    whether the natural keyboard is active
-------------------------------------------------*/

void menu_slot_devices::set_slot_device(device_slot_interface &slot, const char *val)
{
	machine().options().set_value(slot.device().tag()+1, val, OPTION_PRIORITY_CMDLINE);
}

/*-------------------------------------------------
    menu_slot_devices_populate - populates the main
    slot device menu
-------------------------------------------------*/

menu_slot_devices::menu_slot_devices(mame_ui_manager &mui, render_container &container) : menu(mui, container)
{
}

void menu_slot_devices::populate(float &customtop, float &custombottom)
{
	// cycle through all devices for this system
	for (device_slot_interface &slot : slot_interface_iterator(machine().root_device()))
	{
		// does this slot have any selectable options?
		bool has_selectable_options = slot.has_selectable_options();

		// name this option
		std::string opt_name("------");
		const device_slot_option *option = slot_get_current_option(slot);
		if (option)
		{
			opt_name = has_selectable_options
				? option->name()
				: string_format(_("%s [internal]"), option->name());
		}

		// choose item flags
		uint32_t item_flags = has_selectable_options
			? FLAG_LEFT_ARROW | FLAG_RIGHT_ARROW
			: FLAG_DISABLE;

		item_append(slot.device().tag() + 1, opt_name, item_flags, (void *)&slot);
	}
	item_append(menu_item_type::SEPARATOR);
	item_append(_("Reset"), "", 0, (void *)1);
}

menu_slot_devices::~menu_slot_devices()
{
}

/*-------------------------------------------------
    menu_slot_devices - menu that
-------------------------------------------------*/

void menu_slot_devices::handle()
{
	/* process the menu */
	const event *menu_event = process(0);

	if (menu_event != nullptr && menu_event->itemref != nullptr)
	{
		if ((uintptr_t)menu_event->itemref == 1 && menu_event->iptkey == IPT_UI_SELECT)
		{
			machine().schedule_hard_reset();
		}
		else if (menu_event->iptkey == IPT_UI_LEFT || menu_event->iptkey == IPT_UI_RIGHT)
		{
			device_slot_interface *slot = (device_slot_interface *)menu_event->itemref;
			const char *val = (menu_event->iptkey == IPT_UI_LEFT) ? slot_get_prev(*slot) : slot_get_next(*slot);
			set_slot_device(*slot, val);
			reset(reset_options::REMEMBER_REF);
		}
		else if (menu_event->iptkey == IPT_UI_SELECT)
		{
			device_slot_interface *slot = (device_slot_interface *)menu_event->itemref;
			device_slot_option *option = slot_get_current_option(*slot);
			if (option)
				menu::stack_push<menu_device_config>(ui(), container(), slot, option);
		}
	}
}

} // namespace ui
