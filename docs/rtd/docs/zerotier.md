# ZeroTier

## Introduction
The robots from MWLC can be managed and/or operated via internet. 
Although the main purpose of the robot is driving autonomously the robot is heavily dependant on the internet connection with the operator.
The operator sends instructions to the robot.  
* The robot sends status information and a video stream in return.  
* Outside the operational window there are data exchanges and updates.  
  
The connection should be reliable, have enough bandwidth, as little delay as possible and must be secured. 
Especially the video stream increases  the connection demands. Enough bandwidth is necessary and the delay must be minimal. 
It's important that non-authorised parties are excluded from accessing the robot.  
Connections hop via a great number of different parties. The services of the different providers must be coordinated in the connections. 
In addition, there are constantly changing circumstances and requirements.  
A robot runs webservices that can be accessed via a browser. To assure security, the robot is only accessible via a Virtual Private Network 
(VPN) from ZeroTier.   
The operators have access to the robot via a ZeroTier VPN that is setup in advance for one or more robots that are managed by the 
VPN-manager.
MWLC set-up a ZeroTier-VPN with the robot as a node. The information to manage the VPN is available for the VPN-manager.  
#### Connections of the robot
The robot has three types of methods to connect to the internet:
* Wireless mobile such as 4g, 5g, LTE  
* WiFi  
* Ethernet   

The connections are managed by the RUT955. See also https://teltonika-networks.com/product/rut955/.  
The default connection is setup to use the WAN-port. The WAN-port of the router is connected to the WAN-connector at the back of the robot. 
The ethernet connection is very useful for updating and data exchange.
If the WAN-port isn’t used the router will connect via 4g. The WiFi-connections are not in use as they are not very reliable in the open 
air in combination with ZeroTier.
Beside this there is a LAN-connection that creates the possibility to troubleshoot the robot.
ZeroTier
ZeroTier is a peer-to-peer VPN. There isn’t a central server. The code is opensource. The usage of ZeroTier is well described in the 
documentation of ZeroTier.
A VPN from ZeroTier is a network between devices, called Nodes. Nodes can be robots or computers from users. A computer node can have 
different users, this is not noticed by the VPN. Its the task of the VPN-manager to decide which computer-nodes have acces.
A VPN from ZeroTier has an administrator, the VPN-manager.
A ZeroTier-network create a VPN between:
One VPN-manager via de ZeroTier Website
One or more robots. One robot can have more nodes on board that all use the VPN.
Zero or more robot-operator computers If the VPN-manager want to operate the robots, he or she has to use a computer which is part of 
the network.
If a VPN-manager is responsible for more robots then all his or her robots can be accessible via the same network.
MWLC creates a ZeroTier-account with a MWLC-email account and creates a network
Cloud-management of MWLC adds robots to this network.
The VPN-manager is responsible for the computers in the network. If a computer has acces to the network, any user of the computer can 
in principal operate the robot. It is important not to have more computers in the network than strictly necessary.
MWLC sets up the ZeroTier network and adds the robots as nodes. The information is handed over to a VPN-manager to add computers, 
as nodes, for the operators. In the credentials document you will find the information to take over management of your network. 
You can of course also setup your own ZeroTier networks.
Most necessary functions are describe in this document but be aware of any changes by ZeroTier itself.

| Step             |   |
| :---         |    ---: |
| Log in - go to [my.zerotier.com](https://my.zerotier.com)     |   |
| Click ‘LOG IN TO ZEROTIER’  | ![](img/zerotier/login_to_zerotier.png) |
| Email: see credentials <br> Password: See credentials | ![](img/zerotier/login_email_password.png) |
| Click ‘Log In’ ||
| Click ‘Networks’ | ![](img/zerotier/hub_menu.png) |





Choose the network from your credentials.  
If it isn’t available or you see other networks, contact MWLC.







You will see the Network screen. This screen has four sections. In this view all sections can be closed using the blue arrows.
You only need the section called ‘Members’.
Add a computer
Adding computers is done in three steps.
The VPN-manager invites the computer-owner to install the ZeroTier-client and send back the nodeID. As option the VPN-manger can also 
send the NetworkID
The computer owner/: - installs the ZeroTier-client on the computer, - asks in the client access to the Network and - informs the 
VPN-manager the node ID of the client.
The VPN-manager authorises the computer.
Ad 1. The invitation
There are no special requirements to the invitation. At the bottom of the section members there is a little tool. By just filling in 
the email-address the proposed candidate gets all necessary information.
Note: if the VPN-manager already knows the nodeID, its possible to just fill this in under ‘MANUALLY ADD MEMBER’ and add the node to 
the network. The other steps are done at once.
Ad 2. Create a new Node
To add a computer to the network, the computer has to run the ZeroTier-client
Install the application from ZeroTier: https://www.zerotier.com/download/

Your computer gets a ‘Node ID’

In the client choose ‘Join network’


Use the networkID received form the VPN-manager


Click ‘Join’

The network will appear in the list on the computer but before having acces the VPN-manager should authorise the acces.
Inform the VPN- about your nodeID so he or she can check if it appears in the network. This can take minutes, up to hours, depending 
on the complexity of the internet routes between your node and the other nodes.
Keep in mind that changing your IPnumber on the internet wil be seen as a trigger to recalculate all internet routes.
Ad 3. Authorisation of the computer by the VPN-manager
The moment the computer has joined the network it si not able to use it until it is authorised.
This authorisation has to be done by the VPN-manager

Log in into ZeroTier and go to the network via the ‘Networks’ option in the top menu and click on the correct networkID.
The network-management screen appears.

Close the ‘Settings’ by clicking the blue up-arrow.


The section ‘Members’ for managing the nodes of the network appears.



























Check if you recognise the nodeID in the column ‘Address’. If this is a familiar node you can authorise this in the column ‘Auth?’.
After the authorisation the node wil get a managed IP.
Robots in network
In this overview you also see the robots in your network. MWLC is responsible te make a robot accessible via the correct VPN.
Notice
More IP-numbers per node
The VPN client makes the computers and robots part of a closed, virtual IP-network. All nodes have two types of IP:
One to connect the computer to the Internet. The physical connection to the internet. E.g. the 4g-router from the robot gets is IPno 
from the SIM-card. If the computer changes its physical internet connection, such as from 4g to a Lan than this number is changed. 
At that moment the ZeroTier connections will be restructured to find the most efficient routes. During this restructuring the computer 
won’t be connected to the VPN. This can take some time.
One to connect the computer to the virtual private network, the managed IPno.  This number can be changed by the user but doesn’t 
effect for longer time the connection. Within the target of the VPN to give control over the robot there is no need for this.
Automatic optimisation of the VPN
The ZeroTier software calculates the most optimal route between the nodes of the network. This is necessary to avoid delay in the 
video-stream. If somewhere in de route changes are made a recalculation hast to be done. 
If a computer is connected to the ZeroTier VPN a request will be first resolved in the VPN before going outside to the internet.
Connection issues
Primary checks
On the client, is there another VPN running?
On the client, are there any other browsers or Tabs running with a connection to the Robot?
On the client, does the client use the correct network ID?
On the client, does the client have internet access?
On the ZeroTier network member section, does the client appear in the network as ‘ONLINE'?
On the ZeroTier network member section, is the client authorised?
Are there any other browsers or tabs running on a other computer with a connection to the Robot?
Changing from connection/wake up/isp switch
If a node changes its physical connection, 4g,cable or WiFi, it can take several minutes to recalculate the connections.
When a computer wakes up from its stand-by mode this might trigger a recalculation of the network.
Some more advanced routers can switch isp to improve speed or provide failover scenario’s. In case of a ZeroTier network this might be 
contra-productive. Most of the time this also means changing the IPno and therefor recalculation of the network.
In order to improve the connection set-up speed it might help to initiate this manually. On a Mac this goes via the terminal with 
the commands:
sudo launchctl unload /Library/LaunchDaemons/com.zerotier.one.plist
sudo launchctl load /Library/LaunchDaemons/com.zerotier.one.plist
Changing maximum transmission unit
In certain cases the MTU may be too large.
You can inspect the mtu of an interface in the terminal with:
sudo ifconfig
Set the mtu using the network interface name.
sudo ifconfig <interface> mtu 1200
This change is temporary and does not survive restarts. Please refer to your operating system to make the change permanent, if possible.