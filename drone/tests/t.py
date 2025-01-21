mode_en = [
    'STABILIZE', 
    'ACRO', 
    'ALT_HOLD', 
    'AUTO', 
    'GUIDED', 
    'LOITER', 
    'RTL', 
    'CIRCLE',  
    'LAND', 
    'DRIFT', 
    'SPORT', 
    'FLIP', 
    'AUTOTUNE', 
    'POSHOLD', 
    'BRAKE', 
    'THROW', 
    'AVOID_ADSB', 
    'GUIDED_NOGPS', 
    'SMART_RTL', 
    'FLOWHOLD', 
    'FOLLOW', 
    'ZIGZAG', 
    'SYSTEMID', 
    'AUTOROTATE', 
    'AUTO_RTL',
    'TURTLE' ]

value = [0,1,2,3,4,5,6,7,9,11,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28]

dizionario = dict(zip(value,mode_en))

print(dizionario)