def start():
    pass

def stop():
    pass

def reset():
    pass


movement_mapping = {	
                'pause' : stop,
    	        'stop' : stop,
		'slow down' : stop,
                'wait' : stop,
                'cancel' : reset,
                'new goal' : reset,
                'continue' : start,
                'start' : start,
                'resume' : start,
                'go' : start,
	    }

def maps():
    rospy.init_node('maps', anonymous=True)

if __name__ == '__main__':
    maps()
