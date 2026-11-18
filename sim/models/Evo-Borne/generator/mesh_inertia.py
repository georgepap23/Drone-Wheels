import numpy as np, struct
import os
MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "meshes") + "/"
def load(f):
    d=open(MESH_DIR+f,'rb').read(); n=struct.unpack('<I',d[80:84])[0]
    return np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))['v'].reshape(-1,3,3).astype(float)*0.001
def props(f):
    """volume, centroid, inertia about centroid per unit density (mesh frame, metres)"""
    T=load(f); a,b,c=T[:,0],T[:,1],T[:,2]
    v=np.einsum('ij,ij->i',a,np.cross(b,c))/6.0; V=v.sum()
    com=(v[:,None]*(a+b+c)/4).sum(0)/V
    # second moments  integral x_i x_j dV over signed tetrahedra (origin,a,b,c)
    S=np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            S[i,j]=(v*(2*(a[:,i]*a[:,j]+b[:,i]*b[:,j]+c[:,i]*c[:,j])+a[:,i]*b[:,j]+a[:,j]*b[:,i]+a[:,i]*c[:,j]+a[:,j]*c[:,i]+b[:,i]*c[:,j]+b[:,j]*c[:,i])/20).sum()
    S-=V*np.outer(com,com)
    I=np.trace(S)*np.eye(3)-S
    return V,com,I/V           # inertia per unit mass
def in_link(f, mass, R, p):
    V,com,Iu=props(f)
    return mass, R@com+p, mass*(R@Iu@R.T), V
def combine(parts):
    m=sum(x[0] for x in parts); c=sum(x[0]*x[1] for x in parts)/m
    I=np.zeros((3,3))
    for mi,ci,Ii,_ in parts:
        d=ci-c; I+=Ii+mi*(d@d*np.eye(3)-np.outer(d,d))
    return m,c,I
def inertial_xml(m,c,I):
    return (f"<pose>{c[0]:.5f} {c[1]:.5f} {c[2]:.5f} 0 0 0</pose>\n        <mass>{m:.5f}</mass>\n        <inertia>\n"
            f"          <ixx>{I[0,0]:.4e}</ixx><ixy>{I[0,1]:.4e}</ixy><ixz>{I[0,2]:.4e}</ixz>\n"
            f"          <iyy>{I[1,1]:.4e}</iyy><iyz>{I[1,2]:.4e}</iyz><izz>{I[2,2]:.4e}</izz>\n        </inertia>")
if __name__=="__main__":
    for f in ["chassis_only.stl","Left_leg_final.stl","Right_leg_final.stl","N20 DC MOTOR.stl","Wheel_morpho.stl","brushless emax series eco ii 2207 v23.stl","gemfan propeller 51466.stl","Mg995.stl"]:
        V,com,Iu=props(f); print(f"{f:45s} V={V*1e9:10.0f} mm3 com(mm)={np.round(com*1000,1)}")
