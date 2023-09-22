from math import sin, cos, atan, atan2, sqrt, pi
import time, typing
import os; os.system(""); os.system("cls")
import operator


def ball(x: float, y: float, z: float, center: tuple[float, float, float], radius: float ) -> float:
  return sqrt( (x-center[0])**2 + (y-center[1])**2 + (z-center[2])**2) - radius
  
def donut(x: float, y: float, z: float, center: tuple[float, float, float], radius: float, thickness: float) -> float:
  return sqrt((sqrt((x-center[0])**2 + (z-center[2])**2) - radius)**2 + (y-center[1])**2) - thickness / 2
  
def cylinder(x: float, y: float, z: float, p1: tuple[float, float, float], p2: tuple[float, float, float], radius: float ) -> float:
  p = tuple( map( operator.sub, p2, p1 ) )
  x, y, z = map( operator.sub, (x, y, z), p1 )
  
  l = sqrt( p[0]**2 + p[1]**2 + p[2]**2)
  θ = atan2( p[1], p[0] )
  ϕ = atan( p[2]/ sqrt(p[1]**2+p[0]**2+0.0000001) )
  
  x_1 = x * cos(θ) + y * sin(θ)
  y_1 = y * cos(θ) - x * sin(θ)
  z_1 = z
  
  x_2 = x_1 * cos(ϕ) + z_1 * sin(ϕ)
  y_2 = y_1
  z_2 = z_1 * cos(ϕ) - x_1 * sin(ϕ)
  
  x, y, z = x_2, y_2, z_2
  
  r = sqrt(y**2+z**2)-radius
  if x < 0:
    return sqrt( x**2 + max(r,0)**2 )
  elif x > l:
    return sqrt( (x-l)**2 + max(r,0)**2 )
  else:
    return r
  
  
def sdf(x: float, y: float, z: float) -> float:
    H = 0.25
    R = 0.7

    return min(
                cylinder( x, y, z, (-R, -H, 0), (R, -H, 0), .025 ),
                cylinder( x, y, z, (0,-H,-R), (0,-H,R), .025 ),
                donut( x, y, z, (0,-H,0), R, 0.1 ),
                
                cylinder( x, y, z, (-R, +H, 0), (R, +H,0), .025 ),
                cylinder( x, y, z, (0, +H, -R), (0,+H,R), .025 ),
                donut( x, y, z, (0,+H,0), R, 0.05 ),
                
                cylinder( x, y, z, (0, -1.5*H, 0), (0, +1.5*H, 0), R/8 ),
                cylinder( x, y, z, (0, -1.5*H, 0), (0, -0.7*H, 0), R/4 ),
                cylinder( x, y, z, (0, +0.7*H, 0), (0, +1.5*H, 0), R/4 ),
              )


Sdf = typing.Callable[[float, float, float], float]
def normal(sdf: Sdf, x: float, y: float, z: float) -> tuple[float, float, float]:
  ε = 0.001
  n_x = sdf(x + ε, y, z) - sdf(x - ε, y, z)
  n_y = sdf(x, y + ε, z) - sdf(x, y - ε, z)
  n_z = sdf(x, y, z + ε) - sdf(x, y, z - ε) 
  norm = sqrt(n_x**2 + n_y**2 + n_z**2)
  return (n_x / norm, n_y / norm, n_z / norm)

def sample(x: float, y: float, frame: int) -> str:
  θ_pov = frame * 10 / 180 * pi;
  ϕ_pov = 15 / 180 * pi;
  
  z = 10
  for _step in range(30):
    
    t_x, t_y, t_z = x, y, z
    t_x, t_y, t_z = t_x * cos(θ_pov) - t_z * sin(θ_pov), t_y, t_x * sin(θ_pov) + t_z * cos(θ_pov)
    t_x, t_y, t_z = t_x, t_y * cos(ϕ_pov) - t_z * sin(ϕ_pov), t_y * sin(ϕ_pov) + t_z * cos(ϕ_pov)
    
    d = sdf(t_x, t_y, t_z)
    if d <= 0.05:
      #print( 'X: {}, Y: {}, Z: {}\n'.format(t_x, t_y, t_z) )
      _, nt_y, nt_z = normal(sdf, t_x, t_y, t_z)
      is_lit = nt_y > 0.15
      is_frosted = nt_z < -0.75

      if is_frosted:
        return '@' if is_lit else '#'
      else:
        return '=' if is_lit else '.'
    else:
      if z < -1:
        break
      z -= d
  
  if DEPTH_DEBUG:
      #print( 'X: {}, Y: {}, Z: {}\n'.format(t_x, y, t_z) )
      return ' ' if z<0 else '{}'.format(round(z)) if z<9.5 else '+'
  else:
      return ' '

frames = []
for frame in range(0,90):
  X_RES = 400
  Y_RES = 100
  DEPTH_DEBUG = False
  
  frame_chars = []
  for y in range(Y_RES,0,-1):
    for x in range(X_RES):
      remapped_x = x / X_RES * 2 - 1
      remapped_y = (y / Y_RES * 2 - 1) * (2 * Y_RES/X_RES)
      frame_chars.append(sample(remapped_x, remapped_y, frame))
    frame_chars.append('\n')
  frames.append( '\033[H' + ''.join(frame_chars) + f'Frame {frame}\n' )
  print(f'Rendering frame {frame}')
 
input("Ready to draw...")
os.system("cls")
for frame in frames:
    print(frame)
    time.sleep(1/10)


  

#     ^ +Y
#     |
#     |
#  ---.-----------> +X
#    /
#   /
#  / +Z
#   