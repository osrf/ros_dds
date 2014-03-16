#!/usr/bin/env python

import sys
import genmsg.template_tools

msg_template_map = {'msg.idl.template': '@NAME@.idl'}
srv_template_map = {'srv.idl.template': '@NAME@.idl'}

if __name__ == "__main__":
    genmsg.template_tools.generate_from_command_line_options(sys.argv,
                                                             msg_template_map,
                                                             srv_template_map)
