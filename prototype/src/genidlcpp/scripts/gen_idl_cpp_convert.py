#!/usr/bin/env python

import argparse
import em
import genmsg.gentools
import genmsg.msg_loader
import genmsg.template_tools
import os

convert_template_map = {
    'msg_convert.h.template': '@NAME@_convert.h'
}

# ${ARG_MSG} ${ARG_PKG} ${MSG_SHORT_NAME} -o "${ARG_GEN_OUTPUT_DIR}/dds_impl" -e ${GENIDLCPP_TEMPLATE_DIR}

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate source code with conversion functions for messages.')
    parser.add_argument('msgfile', help='The path of the message file')
    parser.add_argument('pkgname', help='The name of the package')
    parser.add_argument('-o', help='The output directory if the generated code')
    parser.add_argument('-e', help='The base directory of the templates')
    args = parser.parse_args()

    msg_context = genmsg.msg_loader.MsgContext.create_default()
    full_type_name = genmsg.gentools.compute_full_type_name(args.pkgname, os.path.basename(args.msgfile))
    spec = genmsg.msg_loader.load_msg_from_file(msg_context, args.msgfile, full_type_name)

    for template_file_name, output_file_name in convert_template_map.items():
        template_file = os.path.join(args.e, template_file_name)
        output_file = os.path.join(args.o, output_file_name.replace("@NAME@", spec.short_name))
        g = {
            "spec": spec
        }

        #print('generate_from_template %s %s %s' % (args.msgfile, template_file, output_file))
        with open(output_file, 'w') as ofile:
            # todo, reuse interpreter
            interpreter = em.Interpreter(output=ofile, globals=g, options={em.RAW_OPT: True, em.BUFFERED_OPT: True})
            if not os.path.isfile(template_file):
                raise RuntimeError('Template file %s not found in template dir %s' % (template_file_name, args.e))
            interpreter.file(open(template_file))
            interpreter.shutdown()
