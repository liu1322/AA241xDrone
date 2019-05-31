import picamera

camera = picamera.PiCamera()
camera.color_effects = (128,128)
camera.resolution=(640,480)
camera.capture('cam20.jpg')
