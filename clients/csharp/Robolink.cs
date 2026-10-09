// VerticalBot Studio — C# client (RoboDK-API-like) over WebSocket JSON-RPC. .NET 6+, no dependencies.
// Usage:  var rdk = new Robolink("ws://localhost:20500"); var robot = await rdk.Item("", Robolink.ITEM_TYPE_ROBOT);
//         await robot.MoveJ(new double[]{0,-90,90,0,90,0}); var pose = await robot.Pose();
using System;
using System.Collections.Generic;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading;
using System.Threading.Tasks;

namespace VerticalBotStudio
{
    public class Robolink : IDisposable
    {
        public const int ITEM_TYPE_ANY = -1, ITEM_TYPE_STATION = 1, ITEM_TYPE_ROBOT = 2, ITEM_TYPE_FRAME = 3, ITEM_TYPE_TOOL = 4, ITEM_TYPE_OBJECT = 5, ITEM_TYPE_TARGET = 6, ITEM_TYPE_PROGRAM = 8;
        private readonly ClientWebSocket ws = new ClientWebSocket();
        private int seq = 0;
        private readonly SemaphoreSlim gate = new SemaphoreSlim(1, 1);

        public Robolink(string url = "ws://localhost:20500") { ws.ConnectAsync(new Uri(url), CancellationToken.None).GetAwaiter().GetResult(); }

        public async Task<JsonNode?> Call(string method, string? target = null, params object?[] args)
        {
            var id = Interlocked.Increment(ref seq);
            var req = new JsonObject { ["id"] = id, ["method"] = method, ["target"] = target, ["params"] = new JsonArray(Array.ConvertAll(args, Encode)) };
            await gate.WaitAsync();
            try
            {
                await ws.SendAsync(Encoding.UTF8.GetBytes(req.ToJsonString()), WebSocketMessageType.Text, true, CancellationToken.None);
                var buf = new byte[1 << 20]; var sb = new StringBuilder();
                WebSocketReceiveResult r;
                do { r = await ws.ReceiveAsync(buf, CancellationToken.None); sb.Append(Encoding.UTF8.GetString(buf, 0, r.Count)); } while (!r.EndOfMessage);
                var res = JsonNode.Parse(sb.ToString())!;
                if (res["error"] != null) throw new Exception(res["error"]!.ToString());
                return res["result"];
            }
            finally { gate.Release(); }
        }

        private static JsonNode? Encode(object? v)
        {
            if (v is Item it) return new JsonObject { ["$item"] = it.Id };
            if (v is double[,] m) { var rows = new JsonArray(); for (int r = 0; r < 4; r++) { var row = new JsonArray(); for (int c = 0; c < 4; c++) row.Add(m[r, c]); rows.Add(row); } return new JsonObject { ["$pose"] = rows }; }
            if (v is double[] a) { var arr = new JsonArray(); foreach (var x in a) arr.Add(x); return arr; }
            return v == null ? null : JsonValue.Create(v);
        }

        public async Task<Item> Item(string name, int type = ITEM_TYPE_ANY) => new Item(this, await Call("Item", null, name, type));
        public async Task<Item> AddFrame(string name, Item? parent = null) => new Item(this, await Call("AddFrame", null, name, parent));
        public async Task<Item> AddTarget(string name, Item? parent = null, Item? robot = null) => new Item(this, await Call("AddTarget", null, name, parent, robot));
        public async Task<Item> AddProgram(string name, Item? robot = null) => new Item(this, await Call("AddProgram", null, name, robot));
        public async Task<Item> AddRobot(string library) => new Item(this, await Call("AddRobot", null, library));
        public async Task<string> Version() => (await Call("Version"))!.ToString();
        public async Task ShowMessage(string msg, bool popup = true) => await Call("ShowMessage", null, msg, popup);
        public void Dispose() => ws.Dispose();
    }

    public class Item
    {
        private readonly Robolink rdk; public readonly string? Id; public readonly string Name; public readonly int Type;
        public Item(Robolink rdk, JsonNode? node) { this.rdk = rdk; Id = node?["$item"]?.ToString(); Name = node?["name"]?.ToString() ?? ""; Type = node?["type"]?.GetValue<int>() ?? -1; }
        public bool Valid() => Id != null;
        private Task<JsonNode?> C(string m, params object?[] a) => rdk.Call(m, Id, a);
        public async Task<double[]> Joints() { var n = await C("Joints"); return n!.AsArray().Select(x => x!.GetValue<double>()).ToArray(); }
        public Task setJoints(double[] q) => C("setJoints", q);
        public async Task<double[,]> Pose() => ToPose((await C("Pose"))!);
        public Task setPose(double[,] p) => C("setPose", p);
        public Task MoveJ(Item t) => C("MoveJ", t);
        public Task MoveJ(double[] q) => C("MoveJ", q);
        public Task MoveL(Item t) => C("MoveL", t);
        public Task MoveL(double[,] p) => C("MoveL", p);
        public Task setPoseFrame(Item f) => C("setPoseFrame", f);
        public Task setPoseTool(Item t) => C("setPoseTool", t);
        public Task setSpeed(double linear, double joints = -1) => C("setSpeed", linear, joints);
        public async Task<(bool ok, string code)> MakeProgram(string post) { var r = await C("MakeProgram", "", post); return (r![0]!.GetValue<bool>(), r[1]!.ToString()); }
        public async Task<JsonNode?> Update() => await C("Update");
        public Task Delete() => C("Delete");
        public static double[,] ToPose(JsonNode n) { var rows = n["$pose"]!.AsArray(); var m = new double[4, 4]; for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) m[r, c] = rows[r]![c]!.GetValue<double>(); return m; }
    }
}
