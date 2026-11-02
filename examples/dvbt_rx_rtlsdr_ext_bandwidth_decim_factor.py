# this module will be imported in the into your flowgraph
# arg: 0 = 2MHz 1 = 1MHz, 2 = 500Khz, 3 = 333KHz, 4 = 250KHz, 5 = 150KHz, 6 = 125KHz, 7 = 66KHz, 8 = 33KHz, 9 = 25KHz, 10 = 20 KHz

def factor(arg):

#	if arg==0:
#		return 1.000
#	elif arg==1:
#		return 2.000
#	elif arg==2:
#        	return 4.000
#	elif arg==3:
#		return 6.006
#	elif arg==4:
#		return 8.000
#	elif arg==5:
#		return 13.333
#	elif arg==6:
#		return 16.000
#	elif arg==7:
#		return 30.303
#	elif arg==8:
#		return 60.606
#	elif arg==9:
#		return 80.000
#	elif arg==10:
#		return 100.000
#	else:
#		return 1.000
	
	if arg==0:
		return 1.000
	elif arg==1:
		return 2.000
	elif arg==2:
        	return 4.000
	elif arg==3:
		return 6.006
	elif arg==4:
		return 8.000
	elif arg==5:
		return 16.000
	elif arg==6:
		return 30.303
	else:
		return 1.000
		
