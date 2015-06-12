using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KinectOSC
{
    class KinectCoordinates
    {
        public string xOffset { set; get; }
        public string yOffset { set; get; }
        public string zOffset { set; get; }
        public string pitch { set; get; }
        public string roll { set; get; }
        public string yaw { set; get; }
    }

    class MetaConfiguration
    {
        public string port { set; get; }
        public string OSCAddress { set; get; }
        public string XScaling { set; get; }
        public string XOffset { set; get; }
        public string YScaling { set; get; }
        public string YOffset { set; get; }
    }
}
