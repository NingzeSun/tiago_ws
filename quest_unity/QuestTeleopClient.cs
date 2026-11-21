// Quest teleop client for tiago_dc (Unity + Meta XR Core SDK). Replaces the WebXR page
// (src/tiago_dc/tiago_dc/web/client.js): same WebSocket protocol to quest_webxr_node, plus eye
// gaze and face expressions (Quest Pro). Shows the camera mosaic from the bridge on a panel.
//
// Poses go out in the WebXR local-floor axes that the bridge and teleop expect (x right, y up,
// z toward the operator), i.e. Unity's tracking space with z flipped. The bridge URL is read from
// <persistentDataPath>/quest_url.txt, which scripts/quest_connect.sh <usb|wifi ip> unity writes.

using System;
using System.Globalization;
using System.IO;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

[RequireComponent(typeof(OVRCameraRig))]
public class QuestTeleopClient : MonoBehaviour
{
    [Tooltip("Material with the Universal Render Pipeline/Unlit shader, for the camera panel")]
    public Material panelMaterial;

    // Panel 1.6 m x 1.2 m (4:3 mosaic), 1.3 m in front of the origin, centre 1.25 m high (as client.js).
    static readonly Vector3 PanelCenter = new Vector3(0f, 1.25f, 1.3f);
    static readonly Vector2 PanelSize = new Vector2(1.6f, 1.2f);
    static readonly OVRInput.Controller[] Controllers = { OVRInput.Controller.LTouch, OVRInput.Controller.RTouch };
    static readonly string[] Sides = { "left", "right" };

    [Serializable]
    class Reply { public string type; public double t; public string message; }

    OVRCameraRig rig;
    OVRFaceExpressions face;
    OVRPlugin.EyeGazesState eyes;
    Texture2D mosaic;
    string[] faceNames;
    readonly StringBuilder json = new StringBuilder(4096);
    readonly CancellationTokenSource quit = new CancellationTokenSource();
    ClientWebSocket socket;
    Task sending = Task.CompletedTask;
    double rttMs = double.NaN, lastPing;
    bool replaced;

    void Start()
    {
        rig = GetComponent<OVRCameraRig>();
        // These components start face and eye tracking once the permissions are granted.
        face = gameObject.AddComponent<OVRFaceExpressions>();
        foreach (var eye in new[] { OVREyeGaze.EyeId.Left, OVREyeGaze.EyeId.Right })
        {
            var gaze = new GameObject($"{eye}EyeGaze").AddComponent<OVREyeGaze>();
            gaze.transform.SetParent(rig.trackingSpace, false);
            gaze.Eye = eye;
        }
        faceNames = new string[(int)OVRFaceExpressions.FaceExpression.Max];
        for (int i = 0; i < faceNames.Length; i++)
            faceNames[i] = ((OVRFaceExpressions.FaceExpression)i).ToString();

        var panel = GameObject.CreatePrimitive(PrimitiveType.Quad);
        Destroy(panel.GetComponent<Collider>());
        panel.transform.SetParent(rig.trackingSpace, false);
        panel.transform.localPosition = PanelCenter;
        panel.transform.localScale = new Vector3(PanelSize.x, PanelSize.y, 1f);
        mosaic = new Texture2D(2, 2);
        var panelRenderer = panel.GetComponent<Renderer>();
        panelRenderer.material = panelMaterial;
        panelRenderer.material.mainTexture = mosaic;
        var view = rig.centerEyeAnchor.GetComponent<Camera>();
        view.clearFlags = CameraClearFlags.SolidColor;
        view.backgroundColor = new Color(0.02f, 0.035f, 0.06f);

        string file = Path.Combine(Application.persistentDataPath, "quest_url.txt");
        if (!File.Exists(file))
        {
            Debug.LogError($"tiago_dc: no bridge URL; write data/quest_url.txt's URL to {file}");
            return;
        }
        RunSocket(SocketUri(File.ReadAllText(file).Trim()));
    }

    void Update()
    {
        double now = Time.realtimeSinceStartupAsDouble;
        if (now - lastPing > 1.0)
        {
            lastPing = now;
            Send($"{{\"type\":\"ping\",\"t\":{F(now * 1000.0)}}}");
        }
        Send(PosePacket(now));
    }

    void OnDestroy()
    {
        quit.Cancel();
        socket?.Abort();
    }

    // ------------------------------------------------------------------ connection

    static Uri SocketUri(string pageUrl)
    {
        // http(s)://host:port/?token=... (the page URL) -> ws(s)://host:port/ws?token=...
        var page = new Uri(pageUrl);
        return new Uri($"{(page.Scheme == "https" ? "wss" : "ws")}://{page.Authority}/ws{page.Query}");
    }

    async void RunSocket(Uri uri)
    {
        // Continuations run on Unity's main thread, so Update, Send and Receive never overlap.
        while (!quit.IsCancellationRequested && !replaced)
        {
            using (var ws = new ClientWebSocket())
            {
                try
                {
                    await ws.ConnectAsync(uri, quit.Token);
                    socket = ws;
                    string hello = $"{{\"type\":\"hello\",\"client\":\"unity {OVRPlugin.GetSystemHeadsetType()}\"," +
                                   $"\"face_names\":[\"{string.Join("\",\"", faceNames)}\"]}}";
                    sending = ws.SendAsync(Utf8(hello), WebSocketMessageType.Text, true, quit.Token);
                    await sending;
                    await Receive(ws);
                }
                catch (Exception e)
                {
                    if (!quit.IsCancellationRequested)
                        Debug.LogWarning($"tiago_dc: {e.Message}");
                }
                socket = null;
            }
            if (!replaced && !quit.IsCancellationRequested)
                await Task.Delay(2000);
        }
    }

    async Task Receive(ClientWebSocket ws)
    {
        var chunk = new byte[1 << 16];
        var message = new MemoryStream();
        while (ws.State == WebSocketState.Open)
        {
            var result = await ws.ReceiveAsync(new ArraySegment<byte>(chunk), quit.Token);
            if (result.MessageType == WebSocketMessageType.Close)
                return;
            message.Write(chunk, 0, result.Count);
            if (!result.EndOfMessage)
                continue;
            byte[] data = message.ToArray();
            message.SetLength(0);
            if (result.MessageType == WebSocketMessageType.Binary)
            {
                mosaic.LoadImage(data);  // JPEG camera mosaic
                continue;
            }
            var reply = JsonUtility.FromJson<Reply>(Encoding.UTF8.GetString(data));
            if (reply.type == "pong")
                rttMs = Time.realtimeSinceStartupAsDouble * 1000.0 - reply.t;
            else if (reply.type == "error")
            {
                Debug.LogWarning($"tiago_dc: {reply.message}");
                // A newer client took control; reconnecting would take it back (as in client.js).
                replaced = reply.message == "another client took control";
            }
        }
    }

    void Send(string text)
    {
        // One send at a time: a packet that finds the previous one still in flight is skipped.
        if (socket == null || socket.State != WebSocketState.Open || !sending.IsCompleted)
            return;
        sending = socket.SendAsync(Utf8(text), WebSocketMessageType.Text, true, quit.Token);
    }

    static ArraySegment<byte> Utf8(string text) => new ArraySegment<byte>(Encoding.UTF8.GetBytes(text));

    // ------------------------------------------------------------------ packets

    string PosePacket(double now)
    {
        json.Clear();
        json.Append("{\"type\":\"pose\",\"t\":").Append(F(now));
        json.Append(",\"rtt_ms\":").Append(double.IsNaN(rttMs) ? "null" : F(rttMs));
        json.Append(",\"head\":");
        AppendPose(rig.centerEyeAnchor.localPosition, rig.centerEyeAnchor.localRotation);
        json.Append(",\"hands\":{");
        bool first = true;
        for (int i = 0; i < Controllers.Length; i++)
        {
            var c = Controllers[i];
            if (!OVRInput.GetControllerPositionTracked(c))
                continue;
            float trigger = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, c);
            float squeeze = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, c);
            json.Append(first ? "\"" : ",\"").Append(Sides[i]).Append("\":{\"pose\":");
            first = false;
            AppendPose(OVRInput.GetLocalControllerPosition(c), OVRInput.GetLocalControllerRotation(c));
            json.Append(",\"trigger\":").Append(F(trigger)).Append(",\"squeeze\":").Append(F(squeeze));
            // xr-standard order: trigger, squeeze, touchpad, thumbstick click, A/X, B/Y.
            json.Append(",\"buttons\":[").Append(B(trigger > 0.5f)).Append(',').Append(B(squeeze > 0.5f))
                .Append(",false,").Append(B(OVRInput.Get(OVRInput.Button.PrimaryThumbstick, c)))
                .Append(',').Append(B(OVRInput.Get(OVRInput.Button.One, c)))
                .Append(',').Append(B(OVRInput.Get(OVRInput.Button.Two, c))).Append("]}");
        }
        json.Append('}');
        AppendEyes();
        if (face.ValidExpressions)
        {
            json.Append(",\"face\":[");
            for (int i = 0; i < faceNames.Length; i++)
                json.Append(i > 0 ? "," : "").Append(F(face[(OVRFaceExpressions.FaceExpression)i]));
            json.Append(']');
        }
        return json.Append('}').ToString();
    }

    void AppendEyes()
    {
        if (!OVRPlugin.GetEyeGazesState(OVRPlugin.Step.Render, -1, ref eyes))
            return;
        var left = eyes.EyeGazes[(int)OVRPlugin.Eye.Left];
        var right = eyes.EyeGazes[(int)OVRPlugin.Eye.Right];
        if (!left.IsValid || !right.IsValid)
            return;
        OVRPose l = left.Pose.ToOVRPose(), r = right.Pose.ToOVRPose();  // tracking space, Unity axes
        json.Append(",\"eyes\":{\"left\":{\"pose\":");
        AppendPose(l.position, l.orientation);
        json.Append(",\"confidence\":").Append(F(left.Confidence)).Append("},\"right\":{\"pose\":");
        AppendPose(r.position, r.orientation);
        json.Append(",\"confidence\":").Append(F(right.Confidence)).Append("}}");

        // Combined gaze ray on the panel plane: u right, v down, both 0..1 (image convention).
        Vector3 origin = (l.position + r.position) / 2f;
        Vector3 direction = (l.orientation * Vector3.forward + r.orientation * Vector3.forward).normalized;
        if (direction.z < 1e-3f)
            return;
        Vector3 hit = origin + direction * ((PanelCenter.z - origin.z) / direction.z);
        float u = (hit.x - PanelCenter.x) / PanelSize.x + 0.5f;
        float v = 0.5f - (hit.y - PanelCenter.y) / PanelSize.y;
        if (u >= 0f && u <= 1f && v >= 0f && v <= 1f)
            json.Append(",\"gaze_uv\":[").Append(F(u)).Append(',').Append(F(v)).Append(']');
    }

    void AppendPose(Vector3 p, Quaternion q)
    {
        // Unity (left-handed, z forward) -> WebXR (right-handed, z toward the operator): flip z.
        json.Append("{\"position\":[").Append(F(p.x)).Append(',').Append(F(p.y)).Append(',').Append(F(-p.z))
            .Append("],\"orientation\":[").Append(F(-q.x)).Append(',').Append(F(-q.y)).Append(',')
            .Append(F(q.z)).Append(',').Append(F(q.w)).Append("]}");
    }

    static string F(double value) => value.ToString("G7", CultureInfo.InvariantCulture);
    static string B(bool value) => value ? "true" : "false";
}
