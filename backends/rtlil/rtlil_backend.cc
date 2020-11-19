/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  Copyright (C) 2012  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  ---
 *
 *  A very simple and straightforward backend for the RTLIL text
 *  representation.
 *
 */

#include "rtlil_backend.h"
#include "kernel/yosys.h"
#include <errno.h>

USING_YOSYS_NAMESPACE
using namespace RTLIL_BACKEND;
YOSYS_NAMESPACE_BEGIN

void RTLIL_BACKEND::dump_const(std::ostream &f, const RTLIL::Const &data, int width, int offset, bool autoint)
{
	if (width < 0)
		width = data.bits.size() - offset;
	if ((data.flags & RTLIL::CONST_FLAG_STRING) == 0 || width != (int)data.bits.size()) {
		if (width == 32 && autoint) {
			int32_t val = 0;
			for (int i = 0; i < width; i++) {
				log_assert(offset+i < (int)data.bits.size());
				switch (data.bits[offset+i]) {
				case State::S0: break;
				case State::S1: val |= 1 << i; break;
				default: val = -1; break;
				}
			}
			if (val >= 0) {
				f << val;
				return;
			}
		}
		f << width << '\'';
		for (int i = offset+width-1; i >= offset; i--) {
			log_assert(i < (int)data.bits.size());
			switch (data.bits[i]) {
			case State::S0: f << '0'; break;
			case State::S1: f << '1'; break;
			case RTLIL::Sx: f << 'x'; break;
			case RTLIL::Sz: f << 'z'; break;
			case RTLIL::Sa: f << '-'; break;
			case RTLIL::Sm: f << 'm'; break;
			}
		}
	} else {
		f << '"';
		std::string str = data.decode_string();
		for (size_t i = 0; i < str.size(); i++) {
			if (str[i] == '\n')
				f << "\\n";
			else if (str[i] == '\t')
				f << "\\t";
			else if (str[i] < 32)
				f << stringf("\\%03o", str[i]);
			else if (str[i] == '"')
				f << "\\\"";
			else if (str[i] == '\\')
				f << "\\\\";
			else
				f << str[i];
		}
		f << '"';
	}
}

void RTLIL_BACKEND::dump_sigchunk(std::ostream &f, const RTLIL::SigChunk &chunk, bool autoint)
{
	if (chunk.wire == NULL) {
		dump_const(f, chunk.data, chunk.width, chunk.offset, autoint);
	} else {
		if (chunk.width == chunk.wire->width && chunk.offset == 0)
		  f << chunk.wire->name.c_str();
		else if (chunk.width == 1)
		  f << chunk.wire->name.c_str() << " [" << chunk.offset << "]";
		else
		  f << chunk.wire->name.c_str() << " [" << (chunk.offset+chunk.width-1)
		    << ':' << chunk.offset << ']';
	}
}

void RTLIL_BACKEND::dump_sigspec(std::ostream &f, const RTLIL::SigSpec &sig, bool autoint)
{
	if (sig.is_chunk()) {
		dump_sigchunk(f, sig.as_chunk(), autoint);
	} else {
		f << "{ ";
		for (auto it = sig.chunks().rbegin(); it != sig.chunks().rend(); ++it) {
			dump_sigchunk(f, *it, false);
			f << ' ';
		}
		f << '}';
	}
}

void RTLIL_BACKEND::dump_wire(std::ostream &f, const std::string &indent, const RTLIL::Wire *wire)
{
	for (auto &it : wire->attributes) {
	  f << indent << "attribute " << it.first.c_str() << ' ';
		dump_const(f, it.second);
		f << '\n';
	}

  for (auto i = 0u; i < ID::kMaxNumBoolIds; ++i) {
    if (wire->bool_attributes.test(i)) {
      f << indent << "attribute "
        << ID::attribute_name(static_cast<ID::BoolId>(i)) << " 1\n";
    }
  }

	f << indent << "wire ";
	if (wire->width != 1)
	  f << "width " << wire->width << ' ';
	if (wire->upto)
	  f << "upto ";
	if (wire->start_offset != 0)
	  f << "offset " << wire->start_offset << ' ';
	if (wire->port_input && !wire->port_output)
	  f << "input " << wire->port_id << ' ';
	if (!wire->port_input && wire->port_output)
	  f << "output " << wire->port_id << ' ';
	if (wire->port_input && wire->port_output)
	  f << "inout " << wire->port_id << ' ';
	if (wire->is_signed)
		f << "signed ";
	f << wire->name.c_str() << '\n';
}

void RTLIL_BACKEND::dump_memory(std::ostream &f, const std::string &indent, const RTLIL::Memory *memory)
{
	for (auto &it : memory->attributes) {
	  f << indent << "attribute " << it.first.c_str() << ' ';
		dump_const(f, it.second);
		f << '\n';
	}
	for (auto i = 0u; i < ID::kMaxNumBoolIds; ++i) {
    if (memory->bool_attributes.test(i)) {
      f << indent << "attribute "
        << ID::attribute_name(static_cast<ID::BoolId>(i)) << " 1\n";
    }
  }
	f << indent << "memory ";
	if (memory->width != 1)
		f << "width " << memory->width << ' ';
	if (memory->size != 0)
		f << "size " << memory->size << ' ';
	if (memory->start_offset != 0)
		f << "offset " << memory->start_offset << ' ';
	f << memory->name.c_str() << '\n';
}

void RTLIL_BACKEND::dump_cell(std::ostream &f, const std::string &indent, const RTLIL::Cell *cell)
{
	for (auto &it : cell->attributes) {
		f << indent << "attribute " << it.first.c_str() << ' ';
		dump_const(f, it.second);
    f << '\n';
	}
	for (auto i = 0u; i < ID::kMaxNumBoolIds; ++i) {
    if (cell->bool_attributes.test(i)) {
      f << indent << "attribute "
        << ID::attribute_name(static_cast<ID::BoolId>(i)) << " 1\n";
    }
  }
	f << indent << "cell " << cell->type.c_str()
	  << ' ' << cell->name.c_str() << '\n';
	for (auto &it : cell->parameters) {
		f << indent << ' parameter'
		  << ((it.second.flags & RTLIL::CONST_FLAG_SIGNED) != 0 ? " signed" : "")
		  << ((it.second.flags & RTLIL::CONST_FLAG_REAL) != 0 ? " real" : "")
		  << ' ' << it.first.c_str() << ' ';
		dump_const(f, it.second);
    f << '\n';
	}
	for (auto &it : cell->connections()) {
		f << indent << "  connect " << it.first.c_str() << ' ';
		dump_sigspec(f, it.second);
    f << '\n';
	}
	f << indent << "end\n";
}

void RTLIL_BACKEND::dump_proc_case_body(std::ostream &f, const std::string &indent, const RTLIL::CaseRule *cs)
{
	for (auto it = cs->actions.begin(); it != cs->actions.end(); ++it)
	{
		f << indent << "assign ";
		dump_sigspec(f, it->first);
		f << ' ';
		dump_sigspec(f, it->second);
    f << '\n';
	}

	for (auto it = cs->switches.begin(); it != cs->switches.end(); ++it)
		dump_proc_switch(f, indent, *it);
}

void RTLIL_BACKEND::dump_proc_switch(std::ostream &f, const std::string &indent, const RTLIL::SwitchRule *sw)
{
	for (auto it = sw->attributes.begin(); it != sw->attributes.end(); ++it) {
		f << indent << "attribute " << it->first.c_str() << ' ';
		dump_const(f, it->second);
    f << '\n';
	}

	f << indent << "switch ";
	dump_sigspec(f, sw->signal);
  f << '\n';

	for (auto it = sw->cases.begin(); it != sw->cases.end(); ++it)
	{
		for (auto ait = (*it)->attributes.begin(); ait != (*it)->attributes.end(); ++ait) {
		  f << indent << "  attribute " << ait->first.c_str() << ' ';
			dump_const(f, ait->second);
	    f << '\n';
		}
		f << indent << "  case ";
		for (size_t i = 0; i < (*it)->compare.size(); i++) {
			if (i > 0)
				f << " , ";
			dump_sigspec(f, (*it)->compare[i]);
		}
    f << '\n';

		dump_proc_case_body(f, indent + "    ", *it);
	}

	f << indent << "end\n";
}

void RTLIL_BACKEND::dump_proc_sync(std::ostream &f, const std::string &indent, const RTLIL::SyncRule *sy)
{
	f << indent << "sync ";
	switch (sy->type) {
	case RTLIL::ST0: f << "low ";
	if (0) case RTLIL::ST1: f << "high ";
	if (0) case RTLIL::STp: f << "posedge ";
	if (0) case RTLIL::STn: f << "negedge ";
	if (0) case RTLIL::STe: f << "edge ";
		dump_sigspec(f, sy->signal);
		f << '\n';
		break;
	case RTLIL::STa: f << "always\n"; break;
	case RTLIL::STg: f << "global\n"; break;
	case RTLIL::STi: f << "init\n"; break;
	}

	for (auto it = sy->actions.begin(); it != sy->actions.end(); ++it) {
		f << indent << "  update ";
		dump_sigspec(f, it->first);
		f << ' ';
		dump_sigspec(f, it->second);
		f << '\n';
	}
}

void RTLIL_BACKEND::dump_proc(std::ostream &f, const std::string &indent, const RTLIL::Process *proc)
{
	for (auto it = proc->attributes.begin(); it != proc->attributes.end(); ++it) {
		f << indent << "attribute " << it->first.c_str() << ' ';
		dump_const(f, it->second);
		f << '\n';
	}
	f << indent <<  "process " << proc->name.c_str() << '\n';
	const auto next_indent = indent + "  ";
	dump_proc_case_body(f, next_indent, &proc->root_case);
	for (auto it = proc->syncs.begin(); it != proc->syncs.end(); ++it)
		dump_proc_sync(f, next_indent, *it);
	f << indent << "end\n";
}

void RTLIL_BACKEND::dump_conn(std::ostream &f, const std::string &indent, const RTLIL::SigSpec &left, const RTLIL::SigSpec &right)
{
	f << indent << "connect ";
	dump_sigspec(f, left);
	f << ' ';
	dump_sigspec(f, right);
	f << '\n';
}

void RTLIL_BACKEND::dump_module(std::ostream &f, const std::string &indent, RTLIL::Module *module, RTLIL::Design *design, bool only_selected, bool flag_m, bool flag_n)
{
	bool print_header = flag_m || design->selected_whole_module(module->name);
	bool print_body = !flag_n || !design->selected_whole_module(module->name);

	if (print_header)
	{
		for (auto it = module->attributes.begin(); it != module->attributes.end(); ++it) {
			f << indent << "attribute " << it->first.c_str() << ' ';
			dump_const(f, it->second);
			f << '\n';
		}

		f << indent << "module " << module->name.c_str() << '\n';

		if (!module->avail_parameters.empty()) {
			if (only_selected)
				f << '\n';
			for (const auto &p : module->avail_parameters) {
				const auto &it = module->parameter_default_values.find(p);
				if (it == module->parameter_default_values.end()) {
					f << indent << "  parameter " << p.c_str() << '\n';
				} else {
					f << indent << "  parameter " << p.c_str() << ' ';
					dump_const(f, it->second);
					f << '\n';
				}
			}
		}
	}

	if (print_body)
	{
	  const auto next_indent = indent + "  ";

		for (auto it : module->wires())
			if (!only_selected || design->selected(module, it)) {
				if (only_selected)
					f << '\n';
				dump_wire(f, next_indent, it);
			}

		for (auto it : module->memories)
			if (!only_selected || design->selected(module, it.second)) {
				if (only_selected)
					f << '\n';
				dump_memory(f, next_indent, it.second);
			}

		for (auto it : module->cells())
			if (!only_selected || design->selected(module, it)) {
				if (only_selected)
					f << '\n';
				dump_cell(f, next_indent, it);
			}

		for (auto it : module->processes)
			if (!only_selected || design->selected(module, it.second)) {
				if (only_selected)
					f << '\n';
				dump_proc(f, next_indent, it.second);
			}

		bool first_conn_line = true;
		for (auto it = module->connections().begin(); it != module->connections().end(); ++it) {
			bool show_conn = !only_selected;
			if (only_selected) {
				RTLIL::SigSpec sigs = it->first;
				sigs.append(it->second);
				for (auto &c : sigs.chunks()) {
					if (c.wire == NULL || !design->selected(module, c.wire))
						continue;
					show_conn = true;
				}
			}
			if (show_conn) {
				if (only_selected && first_conn_line)
					f << '\n';
				dump_conn(f, next_indent, it->first, it->second);
				first_conn_line = false;
			}
		}
	}

	if (print_header)
		f << indent << "end\n";
}

void RTLIL_BACKEND::dump_design(std::ostream &f, RTLIL::Design *design, bool only_selected, bool flag_m, bool flag_n)
{
	int init_autoidx = autoidx;

	if (!flag_m) {
		int count_selected_mods = 0;
		for (auto module : design->modules()) {
			if (design->selected_whole_module(module->name))
				flag_m = true;
			if (design->selected(module))
				count_selected_mods++;
		}
		if (count_selected_mods > 1)
			flag_m = true;
	}

	if (!only_selected || flag_m) {
		if (only_selected)
			f << '\n';
		f << "autoidx " << autoidx << '\n';
	}

	const std::string empty_indent;
	for (auto module : design->modules()) {
		if (!only_selected || design->selected(module)) {
			if (only_selected)
				f << '\n';
			dump_module(f, empty_indent, module, design, only_selected, flag_m, flag_n);
		}
	}

	log_assert(init_autoidx == autoidx);
}

YOSYS_NAMESPACE_END
PRIVATE_NAMESPACE_BEGIN

struct RTLILBackend : public Backend {
	RTLILBackend() : Backend("rtlil", "write design to RTLIL file") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    write_rtlil [filename]\n");
		log("\n");
		log("Write the current design to an RTLIL file. (RTLIL is a text representation\n");
		log("of a design in yosys's internal format.)\n");
		log("\n");
		log("    -selected\n");
		log("        only write selected parts of the design.\n");
		log("\n");
	}
	void execute(std::ostream *&f, std::string filename, std::vector<std::string> args, RTLIL::Design *design) override
	{
		bool selected = false;

		log_header(design, "Executing RTLIL backend.\n");

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			std::string arg = args[argidx];
			if (arg == "-selected") {
				selected = true;
				continue;
			}
			break;
		}
		extra_args(f, filename, args, argidx);

		design->sort();

		log("Output filename: %s\n", filename.c_str());
		*f << stringf("# Generated by %s\n", yosys_version_str);
		RTLIL_BACKEND::dump_design(*f, design, selected, true, false);
	}
} RTLILBackend;

struct IlangBackend : public Backend {
	IlangBackend() : Backend("ilang", "(deprecated) alias of write_rtlil") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("See `help write_rtlil`.\n");
		log("\n");
	}
	void execute(std::ostream *&f, std::string filename, std::vector<std::string> args, RTLIL::Design *design) override
	{
		RTLILBackend.execute(f, filename, args, design);
	}
} IlangBackend;

struct DumpPass : public Pass {
	DumpPass() : Pass("dump", "print parts of the design in RTLIL format") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    dump [options] [selection]\n");
		log("\n");
		log("Write the selected parts of the design to the console or specified file in\n");
		log("RTLIL format.\n");
		log("\n");
		log("    -m\n");
		log("        also dump the module headers, even if only parts of a single\n");
		log("        module is selected\n");
		log("\n");
		log("    -n\n");
		log("        only dump the module headers if the entire module is selected\n");
		log("\n");
		log("    -o <filename>\n");
		log("        write to the specified file.\n");
		log("\n");
		log("    -a <filename>\n");
		log("        like -outfile but append instead of overwrite\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		std::string filename;
		bool flag_m = false, flag_n = false, append = false;

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++)
		{
			std::string arg = args[argidx];
			if ((arg == "-o" || arg == "-outfile") && argidx+1 < args.size()) {
				filename = args[++argidx];
				append = false;
				continue;
			}
			if ((arg == "-a" || arg == "-append") && argidx+1 < args.size()) {
				filename = args[++argidx];
				append = true;
				continue;
			}
			if (arg == "-m") {
				flag_m = true;
				continue;
			}
			if (arg == "-n") {
				flag_n = true;
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		std::ostream *f;
		std::stringstream buf;

		if (!filename.empty()) {
			rewrite_filename(filename);
			std::ofstream *ff = new std::ofstream;
			ff->open(filename.c_str(), append ? std::ofstream::app : std::ofstream::trunc);
			if (ff->fail()) {
				delete ff;
				log_error("Can't open file `%s' for writing: %s\n", filename.c_str(), strerror(errno));
			}
			f = ff;
		} else {
			f = &buf;
		}

		RTLIL_BACKEND::dump_design(*f, design, true, flag_m, flag_n);

		if (!filename.empty()) {
			delete f;
		} else {
			log("%s", buf.str().c_str());
		}
	}
} DumpPass;

PRIVATE_NAMESPACE_END
