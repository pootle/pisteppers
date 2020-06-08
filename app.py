#!/usr/bin/python3
from pootlestuff import webserv
import time
abs_start=time.time()
abs_thread=time.thread_time()
abs_process=time.process_time()
webserv.runmain()
print('Execution summary: elapsed: %7.2f, process: %7.2f, thread: %7.2f' % (time.time()-abs_start, time.process_time()-abs_process, time.thread_time()-abs_thread))