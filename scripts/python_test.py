#!/usr/bin/env python3
import time
for i in range(10):
    time.sleep(0.4)
    print('\r'," "+ \ 
        str(i).ljust(10)+" " \
        str(i).ljust(10)+" " \
        str(i).ljust(10)+" " \
        str(i).ljust(10)+" " \
        str(i).ljust(10)+" " \
        str(i).ljust(10)+" " \
        str(i).ljust(10)+" " \
        str(i).ljust(10)+" " \
        str(i).ljust(10)+" " \
        ,end='')