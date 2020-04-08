import pyglet
from pyglet.gl import * # opengl functions
from pyglet.window import key
import numpy as np
from math import pi, sin, cos, atan, atan2, sqrt

framesPerSecond = 60


class bot():
    def __init__(self, x, y, rot=0, radius=100, viewingWheelWidth=36, viewingWheelHeight=14): # rot in degrees
        # tangible, real world values
        self.x = x
        self.y = y
        self.rot = rot
        self.radius = radius
        self.goalPosition = (self.x, self.y)
        self.goalRotation = self.rot

        self.tWheelVel = 0  # top wheel
        self.blWheelVel = 0 # bottom left wheel
        self.brWheelVel = 0 # bottom right wheel
        self.tWheelTheta = 0 # degrees
        self.blWheelTheta = 0
        self.brWheelTheta = 0

        # simulation values
        self.xVel = 0
        self.yVel = 0
        self.angVel = 0
        self.prevx = self.x
        self.prevy = self.y

        # Values for viewing rect
        self.topWheelSpec = 0
        self.botLeftWheelSpec = 0
        self.botRightWheelSpec = 0
        # wheel rect specs for viewing rect
        self.wheelWidth = viewingWheelWidth
        self.wheelHeight = viewingWheelHeight

    def incRot(self, inc):
        self.rot += inc
        self.rot = self.rot % 360
    def decRot(self, inc):
        self.rot -= inc
        self.rot = self.rot % 360

class userMouse():
    def __init__(self, x=0, y=0, dx=0, dy=0, leftPressed=False):
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.leftPressed = leftPressed

    def leftClickOn(self):
        self.leftPressed = True
    def leftClickOff(self):
        self.leftPressed = False


class userKeyboard():
    def __init__(self, zPressed=False, xPressed=False):
        self.zPressed = zPressed
        self.xPressed = xPressed

    def zButtonPressed(self):
        self.zPressed = True
    def zButtonReleased(self):
        self.zPressed = False
    def xButtonPressed(self):
        self.xPressed = True
    def xButtonReleased(self):
        self.xPressed = False

# create window
window = pyglet.window.Window(width=1000, height=1000)

# initialize classes
bot = bot(window.width/2, window.height/2)
mouse = userMouse()
keyboard = userKeyboard()

# background color
glClearColor(0.2, 0.3, 0.5, 1.0)

# viewing rectangle
rectXOffset = 10
rectYOffset = 10
rectWidth = 250
rectHeight = 250
rectCenter = (rectWidth/2+rectXOffset, window.height - (rectHeight/2+rectYOffset))

viewingRect = pyglet.graphics.vertex_list(4,
    ('v2i', (rectXOffset, window.height-(rectYOffset+rectHeight),
             rectXOffset+rectWidth, window.height-(rectYOffset+rectHeight),
             rectXOffset+rectWidth, window.height-rectYOffset,
             rectXOffset, window.height-rectYOffset)),
    ('c3B', (0, 0, 0,
             0, 0, 0,
             0, 0, 0,
             0, 0, 0)))


# topWheel = pyglet.graphics.vertex_list(4,
#     ('v2i', makeRectCenter(rectCenter[0], rectCenter[1]+bot.radius*2/3, wheelWidth, wheelHeight, pi/4)),
#     ('c3B', (190, 190, 190,
#              190, 190, 190,
#              190, 190, 190,
#              190, 190, 190)))

# helper function to easily make rectangles
def makeRectCenter(x, y, w, h, rot):  # rotation in degrees
    nomAngle = atan(h/w)
    inscribedRadius = sqrt((h/2)**2 + (w/2)**2)
    return (int(x + inscribedRadius*cos(rot*pi/180+pi+nomAngle)), int(y + inscribedRadius*sin(rot*pi/180+pi+nomAngle)),
           int(x + inscribedRadius*cos(rot*pi/180-nomAngle)), int(y + inscribedRadius*sin(rot*pi/180-nomAngle)),
           int(x + inscribedRadius*cos(rot*pi/180+nomAngle)), int(y + inscribedRadius*sin(rot*pi/180+nomAngle)),
           int(x + inscribedRadius*cos(rot*pi/180+pi-nomAngle)), int(y + inscribedRadius*sin(rot*pi/180+pi-nomAngle)))

# Helper function to easily draw circles
def circle(x, y, radius):
    iterations = int(2*radius*pi)
    s = sin(2*pi / iterations)
    c = cos(2*pi / iterations)

    dx, dy = radius, 0

    glBegin(GL_TRIANGLE_FAN)
    glVertex2f(x, y)
    for i in range(iterations+1):
        glVertex2f(x+dx, y+dy)
        dx, dy = (dx*c - dy*s), (dy*c + dx*s)
    glEnd()

def determineIdentifierOffsets(dist1, dist2, angle1, angle2): # angles in degrees
    dot1Offset = (cos((bot.rot-angle1+180)*pi/180)*dist1, sin((bot.rot-angle1+180)*pi/180)*dist1)
    dot2Offset = (cos((bot.rot+angle1)*pi/180)*dist1, sin((bot.rot+angle1)*pi/180)*dist1)
    dot3Offset = (cos((bot.rot+angle2)*pi/180)*dist2, sin((bot.rot+angle2)*pi/180)*dist2)
    return [dot1Offset, dot2Offset, dot3Offset]


def drawCommandArrow():
    arrowAngle = atan2(mouse.y - bot.y, mouse.x - bot.x)*180/pi
    arrowHeadDist = 20
    arrowHeadAngle = 30
    headLeftOffsetx = cos((arrowAngle+180-arrowHeadAngle)*pi/180)*arrowHeadDist
    headLeftOffsety = sin((arrowAngle+180-arrowHeadAngle)*pi/180)*arrowHeadDist
    headRightOffsetx = cos((arrowAngle-180+arrowHeadAngle)*pi/180)*arrowHeadDist
    headRightOffsety = sin((arrowAngle-180+arrowHeadAngle)*pi/180)*arrowHeadDist
    # This may not be the fastest way to draw the arrow
    # only need to instantiate the vertex list once - then can do arrow_vertices.draw() with updated vertex values
    arrow_vertices = pyglet.graphics.vertex_list(6,
        ('v2f', (bot.x, bot.y,
                 mouse.x, mouse.y,
                 mouse.x + headLeftOffsetx, mouse.y + headLeftOffsety,
                 mouse.x, mouse.y,
                 mouse.x + headRightOffsetx, mouse.y + headRightOffsety,
                 mouse.x, mouse.y
                 )),
        ('c3B', (1, 0, 0,
                 1, 0, 0,
                 1, 0, 0,
                 1, 0, 0,
                 1, 0, 0,
                 1, 0, 0))
    )
    arrow_vertices.draw(pyglet.gl.GL_LINES)

def updateRobotVelocity():
    velCap = 100
    # PD controller
    pGain = 1
    dGain = 1000

    posErrorx = bot.goalPosition[0] - bot.x
    posErrory = bot.goalPosition[1] - bot.y

    dErrorx = (bot.prevx - bot.x) / framesPerSecond
    dErrory = (bot.prevy - bot.y) / framesPerSecond

    bot.xVel += pGain * posErrorx + dGain * dErrorx
    bot.yVel += pGain * posErrory + dGain * dErrory

    # # Velocity cap:
    # if bot.xVel > velCap:
    #     bot.xVel = velCap
    # elif bot.xVel < -velCap:
    #     bot.xVel = -velCap
    # if bot.yVel > velCap:
    #     bot.yVel = velCap
    # elif bot.yVel < -velCap:
    #     bot.yVel = -velCap

    bot.prevx = bot.x
    bot.prevy = bot.y

def updateRobotPosition():
    bot.x += bot.xVel / framesPerSecond
    bot.y += bot.yVel / framesPerSecond

def setGoalPosition():
    bot.goalPosition = (mouse.x, mouse.y)

'''
draw viewing box and robot
'''
def drawViewingRectangle():
    # Draw box on top of simulation
    viewingRect.draw(pyglet.gl.GL_QUADS)

    # Draw inside box
    # main circle
    glColor3f(.9,.9,.9)
    circle(rectCenter[0], rectCenter[1], bot.radius*2/3)

    # identifier circles
    glColor3f(.8,.5,.5)
    identifierOffsets = determineIdentifierOffsets(35.87, 33.33, 21.8, 90)
    circle(rectCenter[0] + identifierOffsets[1][0], rectCenter[1] + identifierOffsets[1][1], 6.67)
    circle(rectCenter[0] + identifierOffsets[2][0], rectCenter[1] + identifierOffsets[2][1], 6.67)
    circle(rectCenter[0] + identifierOffsets[0][0], rectCenter[1] + identifierOffsets[0][1], 6.67)

    # wheels
    bot.topWheelSpec = (rectCenter[0] + bot.radius*2/3*cos(bot.rot*pi/180+pi/2), rectCenter[1] + bot.radius*2/3*sin(bot.rot*pi/180+pi/2), bot.rot) # x,y,rot
    bot.botLeftWheelSpec = (rectCenter[0] + bot.radius*2/3*cos(bot.rot*pi/180+7*pi/6), rectCenter[1] + bot.radius*2/3*sin(bot.rot*pi/180+7*pi/6), bot.rot + 120)
    bot.botRightWheelSpec = (rectCenter[0] + bot.radius*2/3*cos(bot.rot*pi/180-pi/6), rectCenter[1] + bot.radius*2/3*sin(bot.rot*pi/180-pi/6), bot.rot + 60)

    topWheel = pyglet.graphics.vertex_list(4,
        ('v2i', makeRectCenter(bot.topWheelSpec[0], bot.topWheelSpec[1], bot.wheelWidth, bot.wheelHeight, bot.topWheelSpec[2])),
        ('c3B', (190, 190, 190,
                 190, 190, 190,
                 190, 190, 190,
                 190, 190, 190)))

    botLeftWheel = pyglet.graphics.vertex_list(4,
        ('v2i', makeRectCenter(bot.botLeftWheelSpec[0], bot.botLeftWheelSpec[1], bot.wheelWidth, bot.wheelHeight, bot.botLeftWheelSpec[2])),
        ('c3B', (190, 190, 190,
                 190, 190, 190,
                 190, 190, 190,
                 190, 190, 190)))

    botRightWheel = pyglet.graphics.vertex_list(4,
        ('v2i', makeRectCenter(bot.botRightWheelSpec[0], bot.botRightWheelSpec[1], bot.wheelWidth, bot.wheelHeight, bot.botRightWheelSpec[2])),
        ('c3B', (190, 190, 190,
                 190, 190, 190,
                 190, 190, 190,
                 190, 190, 190)))

    topWheel.draw(pyglet.gl.GL_QUADS)
    botLeftWheel.draw(pyglet.gl.GL_QUADS)
    botRightWheel.draw(pyglet.gl.GL_QUADS)

def drawViewingRectArrows():
    ## TODO: draw arrows corresponding to velocities and accels at each wheel
    # must be drawn independent of robot rotation
    if bot.tWheelVel > 0:
        bot.tWheelTheta = bot.rot
    else:
        bot.tWheelTheta = bot.rot + 180
    if bot.blWheelVel > 0:
        bot.blWheelTheta = bot.rot + 120
    else:
        bot.blWheelTheta = bot.rot + 120 + 180
    if bot.brWheelVel > 0:
        bot.brWheelTheta = bot.rot + 60
    else:
        bot.brWheelTheta = bot.rot + 60 + 180

    # bot.tWheelTheta = bot.rot
    # bot.blWheelTheta = bot.rot - 120
    # bot.brWheelTheta = bot.rot + 60

    # tWheelDir = bot.tWheelVel/abs(bot.tWheelVel)
    # blWheelDir = bot.blWheelVel/abs(bot.blWheelVel)
    # brWheelDir = bot.brWheelVel/abs(bot.brWheelVel)
    arrowScale = .8
    tWheelArrowLoc = (bot.topWheelSpec[0] + arrowScale * abs(bot.tWheelVel) * cos(bot.tWheelTheta*pi/180), bot.topWheelSpec[1] + arrowScale * abs(bot.tWheelVel) * sin(bot.tWheelTheta*pi/180))
    blWheelArrowLoc = (bot.botLeftWheelSpec[0] + arrowScale * abs(bot.blWheelVel) * cos(bot.blWheelTheta*pi/180), bot.botLeftWheelSpec[1] + arrowScale * abs(bot.blWheelVel) * sin(bot.blWheelTheta*pi/180))
    brWheelArrowLoc = (bot.botRightWheelSpec[0] + arrowScale * abs(bot.brWheelVel) * cos(bot.brWheelTheta*pi/180), bot.botRightWheelSpec[1] + arrowScale * abs(bot.brWheelVel) * sin(bot.brWheelTheta*pi/180))

    arrowHeadDist = 20
    arrowHeadAngle = 30

    tHeadLeftOffsetx = cos((bot.tWheelTheta+180-arrowHeadAngle)*pi/180)*arrowHeadDist
    tHeadLeftOffsety = sin((bot.tWheelTheta+180-arrowHeadAngle)*pi/180)*arrowHeadDist
    tHeadRightOffsetx = cos((bot.tWheelTheta-180+arrowHeadAngle)*pi/180)*arrowHeadDist
    tHeadRightOffsety = sin((bot.tWheelTheta-180+arrowHeadAngle)*pi/180)*arrowHeadDist

    blHeadLeftOffsetx = cos((bot.blWheelTheta+180-arrowHeadAngle)*pi/180)*arrowHeadDist
    blHeadLeftOffsety = sin((bot.blWheelTheta+180-arrowHeadAngle)*pi/180)*arrowHeadDist
    blHeadRightOffsetx = cos((bot.blWheelTheta-180+arrowHeadAngle)*pi/180)*arrowHeadDist
    blHeadRightOffsety = sin((bot.blWheelTheta-180+arrowHeadAngle)*pi/180)*arrowHeadDist

    brHeadLeftOffsetx = cos((bot.brWheelTheta+180-arrowHeadAngle)*pi/180)*arrowHeadDist
    brHeadLeftOffsety = sin((bot.brWheelTheta+180-arrowHeadAngle)*pi/180)*arrowHeadDist
    brHeadRightOffsetx = cos((bot.brWheelTheta-180+arrowHeadAngle)*pi/180)*arrowHeadDist
    brHeadRightOffsety = sin((bot.brWheelTheta-180+arrowHeadAngle)*pi/180)*arrowHeadDist

    tWheelArrowVertices = pyglet.graphics.vertex_list(6,
        ('v2f', (bot.topWheelSpec[0], bot.topWheelSpec[1],
                 tWheelArrowLoc[0], tWheelArrowLoc[1],
                 tWheelArrowLoc[0] + tHeadLeftOffsetx, tWheelArrowLoc[1] + tHeadLeftOffsety,
                 tWheelArrowLoc[0], tWheelArrowLoc[1],
                 tWheelArrowLoc[0] + tHeadRightOffsetx, tWheelArrowLoc[1] + tHeadRightOffsety,
                 tWheelArrowLoc[0], tWheelArrowLoc[1]
                 )),
        ('c3B', (0, 200, 255,
                 0, 200, 255,
                 0, 200, 255,
                 0, 200, 255,
                 0, 200, 255,
                 0, 200, 255)))
    blWheelArrowVertices = pyglet.graphics.vertex_list(6,
        ('v2f', (bot.botLeftWheelSpec[0], bot.botLeftWheelSpec[1],
                 blWheelArrowLoc[0], blWheelArrowLoc[1],
                 blWheelArrowLoc[0] + blHeadLeftOffsetx, blWheelArrowLoc[1] + blHeadLeftOffsety,
                 blWheelArrowLoc[0], blWheelArrowLoc[1],
                 blWheelArrowLoc[0] + blHeadRightOffsetx, blWheelArrowLoc[1] + blHeadRightOffsety,
                 blWheelArrowLoc[0], blWheelArrowLoc[1]
                 )),
        ('c3B', (0, 200, 255,
                 0, 200, 255,
                 0, 200, 255,
                 0, 200, 255,
                 0, 200, 255,
                 0, 200, 255)))
    brWheelArrowVertices = pyglet.graphics.vertex_list(6,
        ('v2f', (bot.botRightWheelSpec[0], bot.botRightWheelSpec[1],
                 brWheelArrowLoc[0], brWheelArrowLoc[1],
                 brWheelArrowLoc[0] + brHeadLeftOffsetx, brWheelArrowLoc[1] + brHeadLeftOffsety,
                 brWheelArrowLoc[0], brWheelArrowLoc[1],
                 brWheelArrowLoc[0] + brHeadRightOffsetx, brWheelArrowLoc[1] + brHeadRightOffsety,
                 brWheelArrowLoc[0], brWheelArrowLoc[1]
                 )),
        ('c3B', (0, 200, 255,
                 0, 200, 255,
                 0, 200, 255,
                 0, 200, 255,
                 0, 200, 255,
                 0, 200, 255)))

    tWheelArrowVertices.draw(pyglet.gl.GL_LINES)
    blWheelArrowVertices.draw(pyglet.gl.GL_LINES)
    brWheelArrowVertices.draw(pyglet.gl.GL_LINES)
    pass

def drawRobot():
    glColor3f(1,1,1)

    circle(bot.x, bot.y, bot.radius)

    glColor3f(.8,.5,.5)

    identifierOffsets = determineIdentifierOffsets(53.8, 50, 21.8, 90)
    circle(bot.x + identifierOffsets[0][0], bot.y + identifierOffsets[0][1], 10)
    circle(bot.x + identifierOffsets[1][0], bot.y + identifierOffsets[1][1], 10)
    circle(bot.x + identifierOffsets[2][0], bot.y + identifierOffsets[2][1], 10)

'''
Calculate wheel velocities given robot velocity
'''
def calcWheelVels():
    # We know desired robot velocity (simulation velocity is always equal to goal velocity unless capped)
    # so can calculate individual wheel goal velocities that make it so the robot is moving at the overall goal velocity
    alpha1 = pi/2 - bot.rot*pi/180
    alpha2 = alpha1 + 2*pi/3
    alpha3 = alpha2 + 2*pi/3

    # wheelVelsMatrix = np.array([[bot.tWheelVel],
    #                             [bot.blWheelVel],
    #                             [bot.brWheelVel]])
    botVelsMatrix = np.array([[bot.xVel],
                              [bot.yVel],
                              [bot.angVel]])
    # From wheel vels to bot vels
    wheelForwardMatrix = np.array([[sin(alpha1+pi/2), sin(alpha2 + pi/2), sin(alpha3 + pi/2)],
                                   [cos(alpha1+pi/2), cos(alpha2 + pi/2), cos(alpha3 + pi/2)],
                                   [1, 1, 1]])
    # From bot vels to wheel vels
    # wheelInvMatrix = np.linalg.inv(wheelForwardMatrix)
    # OR USE THIS (found symbolically) - calculating all these trig functions is much faster than computing the inverse every time
    wheelInvMatrix = np.array([[-(cos(alpha2) - cos(alpha3))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), (-(-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) + (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)) - (cos(alpha2) - cos(alpha3))*(sin(alpha1) - sin(alpha2)))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)))), (cos(alpha2)*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))) - (cos(alpha2) - cos(alpha3))*(cos(alpha1)*(-sin(alpha1) + sin(alpha2)) - sin(alpha1)*(-cos(alpha1) + cos(alpha2))))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))))],
    [-(-cos(alpha1) + cos(alpha3))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), ((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)) - (-cos(alpha1) + cos(alpha3))*(sin(alpha1) - sin(alpha2)))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)))), (-cos(alpha1)*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))) - (-cos(alpha1) + cos(alpha3))*(cos(alpha1)*(-sin(alpha1) + sin(alpha2)) - sin(alpha1)*(-cos(alpha1) + cos(alpha2))))/((-cos(alpha1) + cos(alpha2))*((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))))],
    [(-cos(alpha1) + cos(alpha2))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), (sin(alpha1) - sin(alpha2))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2))), (cos(alpha1)*(-sin(alpha1) + sin(alpha2)) - sin(alpha1)*(-cos(alpha1) + cos(alpha2)))/((-cos(alpha1) + cos(alpha2))*(-sin(alpha1) + sin(alpha3)) - (-cos(alpha1) + cos(alpha3))*(-sin(alpha1) + sin(alpha2)))]])

    wheelVelsMatrix = np.dot(wheelInvMatrix, botVelsMatrix)
    bot.tWheelVel = wheelVelsMatrix[0]
    bot.brWheelVel = -wheelVelsMatrix[1]
    bot.blWheelVel = wheelVelsMatrix[2]
    # Can add a PWM conversion here or velocity cap wheels
    # Should probably add PWM conversion in a different function

'''
Mouse and keyboard events
'''
@window.event
def on_mouse_release(x,y,button,mod):
    mouse.leftClickOff()

@window.event
def on_mouse_press(x,y,button,mod):
    mouse.leftClickOn()

# update mouse class location
@window.event
def on_mouse_motion(x,y,dx,dy):
    mouse.x = x
    mouse.y = y
    mouse.dx = dx
    mouse.dy = dy

@window.event
def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
    mouse.x = x
    mouse.y = y
    mouse.dx = dx
    mouse.dy = dy

@window.event
def on_key_press(symbol, mods):
    if symbol == key.Z:
        keyboard.zButtonPressed()
    if symbol == key.X:
        keyboard.xButtonPressed()

@window.event
def on_key_release(symbol, mods):
    if symbol == key.Z:
        keyboard.zButtonReleased()
    if symbol == key.X:
        keyboard.xButtonReleased()

'''
Main loop function
'''
@window.event
def on_draw():
    # Draw order: robot, arrow, viewing box, box arrows
    window.clear() # clearing and redrawing everything each frame may affect performance
    glClear(GL_COLOR_BUFFER_BIT)

    drawRobot()

    if mouse.leftPressed:
        drawCommandArrow()
        setGoalPosition()

    drawViewingRectangle()

    calcWheelVels()
    drawViewingRectArrows()

    updateRobotVelocity() # TODO:  change to update motor velocities or to update by whatever method the motors will be controlled
    updateRobotPosition() # for simulation but should also exist in some form for real life if doing slam (must be different if simulation does not match environment well enough)

    if keyboard.zPressed:
        bot.incRot(4)
        bot.angVel = -400
    else:
        bot.angVel = 0
    if keyboard.xPressed:
        bot.decRot(4)
        bot.angVel = 400


def update(dt):
    pass

pyglet.clock.schedule_interval(update, 1/framesPerSecond)
pyglet.app.run()
