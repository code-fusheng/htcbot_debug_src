'''
Author: code-fusheng
Date: 2024-01-03 00:14:51
LastEditors: code-fusheng 2561035977@qq.com
LastEditTime: 2024-04-26 21:38:18
Description: 
'''
#!/usr/bin/env python2
#-*-coding:utf-8-*-

import parser

# sentence = "$GHFPD,1234,12234.123,90.123,45.55,33.2,23.66,127.1111,78.23,1.1,2.3,6.6,1.5,16,23,2,4*68"
# sentence = "$GNGGA,124216.00,2812.21694085,N,11301.30710367,E,2,00,9999.0,57.0184,M,-16.1671,M,1.0,3330*72"
# sentence = "$GINS,1451,368123.310,34.1966,108.85,80.3,12.3,34.3,1.4,0.32,-22,90.32,11,1,2,3,4,5,1.1,7,*58"
# sentence = "$GPZDA,124216.00,24,03,2024,,*65"
sentence = "$GNRMC,124216.00,A,2812.21694085,N,11301.30710367,E,0.226,276.4,240324,4.0,W,D,C*5E"
# sentence = "$GPVTG,,,,,,,,,A*3F"


res = parser.parse_nmea_sentence(sentence)
print(res)