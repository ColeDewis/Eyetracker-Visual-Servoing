import customtkinter as ctk
from ws_utils.enums import TrackComponentType, ErrorDefinitionType
from utils.utils import *
from geometry_msgs.msg import Transform, Quaternion, Vector3
from scipy.spatial.transform import Rotation
import rospy


class CameraPositionTab(ctk.CTkFrame):
    def __init__(self,parent, root):
        ctk.CTkFrame.__init__(self, root)
        self._fg_color = "transparent"
        #ctk.CTkLabel(self, text="PLEASE").pack()
        self.cam1 = CameraPos(parent, root,1)
        self.cam2 = CameraPos(parent, root,2)

        self.cam1.grid(column=0, row=0, sticky="new", padx=5,pady=5)
        self.cam2.grid(column=1, row=0, sticky="new", padx=5,pady=5)
        self.parent=parent
        
    def publishCamPos(self, not_cam_2, not_cam_1):

        if not not_cam_2:
            self.cam2._sendStart("/camera/cam"+str(self.parent.camIndices[-1])+"/change_pos")
            


        if not not_cam_1:
            self.cam1._sendStart("/camera/cam"+str(self.camIndices[0])+"/change_pos")


class CameraPos(ctk.CTkFrame):
    """_summary_
            Button widget for determining tracker type. extends frame
    """
    def __init__(self, parent, root, header):
        """_summary_
                Initializes widget, establishes buttons
        Args:
            parent (_type_): tells widget where to send callbacks
        """
        ctk.CTkFrame.__init__(self, root)
        self._fg_color = "transparent"
        ctk.CTkLabel(self, text = "Camera " + str(header)).grid(row=0, column=1, columnspan=2, sticky="ew")

        ctk.CTkLabel(self, text="X").grid(column=0, row=1, sticky="ew", padx=5,pady=5)
        self.x = ctk.CTkEntry(self, width=50)
        self.x.grid(column=0, row=2, sticky="ew", padx=5,pady=5)
        self.x.insert(0, "0.6")

        ctk.CTkLabel(self, text="Y").grid(column=1, row=1, sticky="ew", padx=5,pady=5)
        self.y = ctk.CTkEntry(self, width=50)
        self.y.grid(column=1, row=2, sticky="ew", padx=5,pady=5)
        self.y.insert(0, "0.45")

        ctk.CTkLabel(self, text="Z").grid(column=2, row=1, sticky="ew", padx=5,pady=5)
        self.z = ctk.CTkEntry(self, width=50)
        self.z.grid(column=2, row=2, sticky="ew", padx=5,pady=5)
        self.z.insert(0, "0.15") 

        ctk.CTkLabel(self, text="Roll").grid(column=0, row=3, sticky="ew", padx=5,pady=5)
        self.roll = ctk.CTkEntry(self, width=50)
        self.roll.grid(column=0, row=4, sticky="ew", padx=5,pady=5)
        self.roll.insert(0, "-90")

        ctk.CTkLabel(self, text="Pitch").grid(column=1, row=3, sticky="ew", padx=5,pady=5)
        self.pitch = ctk.CTkEntry(self, width=50)
        self.pitch.grid(column=1, row=4, sticky="ew", padx=5,pady=5)
        self.pitch.insert(0, "0")

        ctk.CTkLabel(self, text="Yaw").grid(column=2, row=3, sticky="ew", padx=5,pady=5)
        self.yaw = ctk.CTkEntry(self, width=50)
        self.yaw.grid(column=2, row=4, sticky="ew", padx=5,pady=5)
        self.yaw.insert(0, "-180")

        self.type = ctk.CTkOptionMenu(self,values=["Degrees", "Radians"])
        self.type.grid(row=4, column=3)


    def _getEm(self):
        return(self.type.get(), self.x.get(), self.y.get(), self.z.get(), self.roll.get(), self.pitch.get(), self.yaw.get())
    
    def _sendStart(self, pubString):

        type, x,y,z,r,p,y = self._getEm()
        if type=="Degrees":
            # Create a rotation object from Euler angles specifying axes of rotation
            rot = Rotation.from_euler('xyz', [r,p,y], degrees=True)

        elif type=="Radians":
            rot = Rotation.from_euler('xyz', [r,p,y], degrees=False)

        q = Quaternion()
        q.x, q.y, q.z, q.w = rot.as_quat()

        v = Vector3(x=x,y=y,z=z)
        #v.x, v.y, v.z = x,y,z
        
        t = Transform(translation=v, rotation=q)
        '''t.translation = v
        t.rotation = q'''

        cam_pos_pub = rospy.Publisher(pubString, Transform, queue_size=10)
        '''while cam_pos_pub.get_num_connections() < 1:
            continue'''

        cam_pos_pub.publish(t)
        cam_pos_pub.unregister()


