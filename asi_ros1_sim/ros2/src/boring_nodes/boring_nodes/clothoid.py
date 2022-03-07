import math, sys
from boring_nodes import arc, rk4

def get_step_size(prcnt, k, dkdl, h):
  det1 = k*k - 4.0*prcnt*dkdl
  if det1 > 0:
    det1 = math.sqrt(det1)
    h1 = (-k + det1)/2.0/dkdl
    if h1*h > 0 and abs(h1) < abs(h):
      h = h1
    h2 = (-k - det1)/2.0/dkdl
    if h2*h > 0 and abs(h2) < abs(h):
      h = h2
  return h

def toArcs(x0,y0,q0,k0,dkdl,L,speed):
  if dkdl == 0:
    return [arc.Arc(x0,y0,q0,k0,L,speed)]

  def clothoid_derivs_(l, xy):
    qi = xy[2] + l*(xy[3]+l*xy[4]/2.0)
    return [math.cos(qi), math.sin(qi), xy[3]+l*xy[4], xy[4], 0]

  z = [x0, y0, q0, k0, dkdl]
  l = 0.0
  ltot = 0.0
  Ksat = 1e-8
  done = False
  arcs = []
  prcnt = 2.0
  sgn = 1.0 if L >= 0 else -1.0
  while not done: # Terminate early?
    # h < prcnt% of start radius (abs(prcnt/k))  AND  h < prcnt% of end radius (abs(prcnt/k+h*r))
    k = k0 + l*dkdl
    h0 = prcnt / (Ksat if (abs(k) < Ksat) else abs(k)) * sgn
    h = get_step_size( prcnt/100.0, k, dkdl, h0)
    h = get_step_size(-prcnt/100.0, k, dkdl, h)
    done = (ltot+h)*L >= L*L
    if done: h = L-ltot
    l += h
    ltot += h
    z = rk4.rk4(l, z, h, clothoid_derivs_)
    newarc = arc.Arc(x0, y0, q0, k0, l, speed)
    final = newarc.state(l)
    if done or math.hypot(z[1]-final['y'], z[0]-final['x']) > 0.1: # Add new arc when deviation is greater than 5 cm
      arcs.append(newarc)
      x0 = z[0]
      y0 = z[1]
      q0 += l*(k0 + l*dkdl/2.0)
      k0 += l*dkdl
      z = [x0,y0,q0,k0,dkdl]
      l = 0
  return arcs

def ASIClothoidToArcs(segments):
  arcs = []
  for seg in segments:
    arcs += toArcs(seg['start_x_m'],seg['start_y_m'],seg['start_heading_rad'],
                  seg['start_curvature_inv_m'],seg['delta_curvature_per_length_inv_m2'],
                  seg['length_m'],seg['speed_mps'])
  return arcs

if __name__ == '__main__':
  # import arc, rk4
  import os, json
  from pathlib import Path
  import matplotlib.pyplot as plt
  thisfolder = os.path.dirname(os.path.realpath(__file__))
  x = json.loads(Path(os.path.join(thisfolder,'..','..','..','..','vy','plans','asi_mining_loop_path.object.json')).read_text())
  arcs = ASIClothoidToArcs(x['data']['segments'][0:5])
  states = arc.linspace(arcs,200)
  plt.plot([s['x'] for s in states],[s['y'] for s in states],'.')
  plt.axis('equal');
  plt.show()
