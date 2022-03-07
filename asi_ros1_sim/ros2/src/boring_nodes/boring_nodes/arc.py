import math
SMALL_ANGLE = 0.001
VERY_SMALL_CURVATURE = 1e-6
EPSILON = 1e-12
def pimod(q):
  return ( q + math.pi) % (2 * math.pi ) - math.pi

def sign(q):
  return 0 if q == 0 else( 1 if q > 0 else -1)

def percentAlongLineSegment(x0, y0, x1, y1, x, y):
    dpx = x1 - x0
    dpy = y1 - y0
    distance_squared = dpx*dpx + dpy*dpy
    # If segment is too short to check return a value (2) outside range 0-1
    return 2 if (distance_squared == 0) else ((x - x0)*dpx + (y - y0)*dpy)/distance_squared

class Arc:
  def __init__(self,start_x, start_y, start_heading, curvature, distance, speed):
    self.start_x = start_x
    self.start_y = start_y
    self.start_heading = start_heading
    self.curvature = curvature
    self.distance = distance
    self.speed = speed

  def __str__(self):
    return 'start_x={} start_y={} start_heading={} curvature={} distance={} speed={}'.format(
      self.start_x, self.start_y, self.start_heading, self.curvature, self.distance, self.speed)

  def length(self):
    return abs(self.distance)
    
  def state(self,distance):
    cosine_initial_heading = math.cos(self.start_heading)
    sine_initial_heading = math.sin(self.start_heading)
    if abs(self.curvature) < VERY_SMALL_CURVATURE:
        x = self.start_x + cosine_initial_heading*distance
        y = self.start_y + sine_initial_heading*distance
    elif abs(self.curvature * distance) < SMALL_ANGLE:
        segment_scalar = self.curvature * distance / 2
        x = self.start_x + (cosine_initial_heading - segment_scalar*sine_initial_heading)*distance
        y = self.start_y + (sine_initial_heading + segment_scalar * cosine_initial_heading)*distance
    else:
        x = self.start_x + (math.sin(self.start_heading + self.curvature*distance) - sine_initial_heading)/self.curvature
        y = self.start_y + (cosine_initial_heading - math.cos(self.start_heading + self.curvature * distance))/self.curvature

    return {
      'x':x,
      'y':y,
      'heading':self.start_heading + self.curvature*distance,
      'curvature':self.curvature,
      'speed':self.speed
    }

  def endState(self):
    return self.state(self.distance)

  def center(self):
    return {'x':self.start_x - math.sin(self.start_heading)/self.curvature,
            'y':self.start_y + math.cos(self.start_heading)/self.curvature}

  def percentAlong(self, x, y):
      if abs(self.curvature) < VERY_SMALL_CURVATURE:
          final_state = self.endState()
          return percentAlongLineSegment(self.start_x, self.start_y, final_state['x'], final_state['y'], x, y)
      center = self.center()
      arc_center_x_to_test_x = x - center['x']
      arc_center_y_to_test_y = y - center['y']
      arc_delta_heading = self.curvature*self.distance
      if abs(arc_delta_heading) <= EPSILON:
          return 2; # arc is too short to check, just return a value outside the range 0 - 1
      angle_from_arc_center_to_test_point = math.atan2(arc_center_y_to_test_y, arc_center_x_to_test_x)
      angle_from_arc_center_to_arc_midpoint = self.start_heading + arc_delta_heading/2 + \
          (-math.pi/2 if (self.curvature > 0) else math.pi/2)
      angle_from_arc_center_to_arc_midpoint = pimod(angle_from_arc_center_to_arc_midpoint)

      delta_angle = angle_from_arc_center_to_test_point - angle_from_arc_center_to_arc_midpoint
      qdistc = pimod(delta_angle) / arc_delta_heading
      qdistc_opposite = pimod(delta_angle + math.pi) / arc_delta_heading;
      if abs(qdistc) <= 0.5:
          return qdistc + 0.5; # Line goes from center point of arc to (x,y) to point on arc
      elif abs(qdistc_opposite) <= 0.5:
          return qdistc_opposite + 0.5; # Line goes from (x,y) to center of arc to point on arc
      else:
          # Can't draw a line from (x,y) to center point of arc to arc
          arc_end_x = center['x'] + math.sin(self.start_heading + arc_delta_heading)/self.curvature
          arc_end_y = center['y'] - math.cos(self.start_heading + arc_delta_heading)/self.curvature
          checkside = math.hypot(self.start_x - x, self.start_y - y) < math.hypot(arc_end_x - x, arc_end_y - y)
          choice = [qdistc + 0.5, qdistc_opposite + 0.5]
          return min(choice) if (checkside) else max(choice); # return outside region

def lengthAtFirstPerpendicularPointonPath(arcs, x, y):
    # assuming "arcs" are a (close to) continuous path of lines/arcs then:
    # lengthAtFirstPerpendicularPointonPath will return the length of the first point on "arcs"
    # that is perpendicular to (x,y) Second tuple item is true if (x,y) is not perpendicular
    # to any point on arcs (and x,y is closer to the end than the start of all segments)
    lpast = 0
    for arc in arcs:
        prcnt = arc.percentAlong(x, y)
        if prcnt <= 1 and prcnt >= 0:
            return (lpast + prcnt*arc.length(), False)
        elif prcnt < 0:
            return(lpast, False)
        lpast += arc.length()
    return (lpast,True)

def makeArcPath(x0,y0,q0,path):
  # path list of dictionaries with {'speed':,'distance':,'curvature':}
  segment_start = {'x':x0, 'y':y0, 'heading':q0}
  arcs = []
  for p in path:
    arc = Arc(segment_start['x'],segment_start['y'],segment_start['heading'],
      p.get('curvature',0),p.get('distance',10),p.get('speed',2))
    arcs.append(arc)
    segment_start = arc.endState()
  return arcs

def linspace(arcs,n):
  L = sum([a.distance for a in arcs])
  l = [float(dd)/float(n-1)*L for dd in range(n)]
  return [state(arcs,d) for d in l]

def state(arcs,distance):
  for arc in arcs:
    if distance < arc.length():
      return arc.state(distance)
    distance -= arc.length()
  if len(arcs) > 0:
    return arcs[-1].state(distance+arcs[-1].length())
  return None

def getNearestPointAndPop(path,x,y):
    l,finished = lengthAtFirstPerpendicularPointonPath(path,x,y)
    while len(path) > 1 and l > path[0].length(): # pop a segment
      l -= path[0].length()
      path[:] = path[1:]
    return l,finished

def test():
  path = makeArcPath(0,0,0,[{'distance':50,'curvature':0,'speed':2},
                            {'distance':10*math.pi/2,'curvature':0.1,'speed':2},
                            {'distance':50,'curvature':0,'speed':5},
                            {'distance':10*math.pi/2,'curvature':0.1,'speed':2},
                            {'distance':50,'curvature':0,'speed':5},
                            {'distance':10*math.pi/2,'curvature':0.1,'speed':2},
                            {'distance':50,'curvature':0,'speed':5}])

  l = 0
  dl = 1
  finished = False
  while not finished:
    s = state(path,l)
    l2,finished = getNearestPointAndPop(path,s['x'],s['y'])
    l = l2 + dl

if __name__ == '__main__':
  test()
