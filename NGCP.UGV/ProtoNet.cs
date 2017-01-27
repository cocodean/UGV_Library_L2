using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NGCP.UGV
{
    public class ProtoNet
    {
        #region Callback class

        /// <summary>
        /// Hold all state and behavior data in this callback class
        /// </summary>
        public class ProtonetCallbacks
        {
            public Comnet.Node.EnterDelegate OnEnterCallback;

            public Comnet.Node.ExitDelegate OnExitCallback;

            public Comnet.Node.PingDelegate OnPingCallback;

            public Comnet.Node.PongDelegate OnPongCallback;

            //public Protonet.Node.RawDelegate OnRawCallback;

            public Comnet.Node.VehicleJoystickCommandDelegate OnJoystickCallback;

            public Comnet.Node.VehicleGlobalPositionDelegate OnVehicleGlobalPositionCallback;
        }

        #endregion Callback class

        #region Public Properties

        /// <summary>
        /// Local Node id
        /// </summary>
        public byte NodeID { get; private set; }

        /// <summary>
        /// Node ID of target
        /// </summary>
        public byte TargetNodeID { get; private set; }

        /// <summary>
        /// Call backs of protonet
        /// </summary>
        public ProtonetCallbacks Callbacks;

        #endregion Public Properties

        #region Private Properties

        /// <summary>
        /// Node object of protonet
        /// </summary>
        Comnet.Node Node;

        /// <summary>
        /// Link ID object
        /// </summary>
        sbyte link_id = 0;

        #endregion Private Properties

        #region Constructor

        /// <summary>
        /// Constructor of protonet
        /// </summary>
        /// <param name="NodeID"></param>
        public ProtoNet(byte NodeID)
        {
            //node id of protonet
            this.NodeID = NodeID;
            //Create a protonet node at specified node id
            Node = new Comnet.Node(NodeID);
            //Construct class to hold all callbacks and behaviors
            Callbacks = new ProtonetCallbacks();
        }

        #endregion Constructor

        #region Public Method

        /// <summary>
        /// Start Protonet on serial
        /// </summary>
        public void StartSerial(byte TargetNodeID, uint Baud, string PortName)
        {
            //remember the target id
            this.TargetNodeID = TargetNodeID;
            //Register the methods of the callback class with Protonet
            if (Callbacks.OnEnterCallback != null)
                Node.RegisterEnterEvent(Callbacks.OnEnterCallback);
            if (Callbacks.OnExitCallback != null)
                Node.RegisterExitEvent(Callbacks.OnExitCallback);
            if (Callbacks.OnPingCallback != null)
                Node.RegisterPingEvent(Callbacks.OnPingCallback);
            if (Callbacks.OnPongCallback != null)
                Node.RegisterPongEvent(Callbacks.OnPongCallback);
            if (Callbacks.OnJoystickCallback != null)
                Node.RegisterVehicleJoystickCommandEvent(Callbacks.OnJoystickCallback);
            if (Callbacks.OnVehicleGlobalPositionCallback != null)
                Node.RegisterVehicleGlobalPositionEvent(Callbacks.OnVehicleGlobalPositionCallback);
            //prepare link
            try
            {

                Node.AddZigBeeDatalink(out link_id, 57600, "25");
                //Node.EstablishZigBeeEndpoint(link_id, 4, "0013A2004067BF33");
                Node.EstablishZigBeeEndpoint(link_id, 1, "0013A2004091798F");
            }
            catch (Exception e)
            {
                Console.WriteLine(e.GetType());
            }
            //Start the session handler and begin sending/receiving messages
            Node.Start();
        }

        /// <summary>
        /// Send a state to GCS
        /// </summary>
        /// <param name="state"></param>
        public void SendState(UGVState state)
        {
            double timestamp = (DateTime.Now - UnixTime).TotalMilliseconds;
            Node.SendVehicleGlobalPosition(TargetNodeID, timestamp, NodeID,
                (int)(state.Latitude * 10000000.0), (int)(state.Longitude * 10000000.0), 1, 0, 0, 0);
        }

        static DateTime UnixTime = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

        #endregion Public Method
    }
}
