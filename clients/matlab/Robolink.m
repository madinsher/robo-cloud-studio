classdef Robolink < handle
    % VerticalBot Studio - MATLAB client (RoboDK-API-like) over the TCP JSON-lines API (port 20501).
    %   rdk = Robolink('127.0.0.1', 20501);
    %   robot = rdk.Item('', 2);                 % ITEM_TYPE_ROBOT
    %   rdk.call('MoveJ', robot, {[0 -90 90 0 90 0]});
    %   pose = rdk.Pose(robot);                  % 4x4 matrix (rows)
    %
    % Wire format: items are {"$item": id}, poses {"$pose": [[row1],[row2],[row3],[row4]]}. MATLAB cannot use '$' in
    % struct field names, so requests are written with "$item"/"$pose" (strrep after jsonencode) and replies decode
    % to fields x_item / x_pose (MATLAB's own renaming); the server also accepts x_item / x_pose.
    %
    % The official RoboDK MATLAB API (C:/RoboDK/Matlab) also works unmodified when the studio server runs the RoboDK
    % binary protocol (STUDIO_RDK_PORT=20500, see docs/api.md).
    properties
        t
        seq = 0
    end
    methods
        function obj = Robolink(host, port)
            if nargin < 1, host = '127.0.0.1'; end
            if nargin < 2, port = 20501; end
            obj.t = tcpclient(host, port);
            configureTerminator(obj.t, "LF");
        end
        function res = call(obj, method, target, params)
            if nargin < 3, target = []; end
            if nargin < 4, params = {}; end
            obj.seq = obj.seq + 1;
            enc = cellfun(@Robolink.encode, params, 'UniformOutput', false);
            req = struct('id', obj.seq, 'method', method);
            req.params = enc;
            if ~isempty(target) && isstruct(target) && isfield(target, 'x_item'), req.target = target.x_item; end
            txt = jsonencode(req);
            txt = strrep(strrep(txt, '"x_item":', '"$item":'), '"x_pose":', '"$pose":');
            writeline(obj.t, txt);
            line = readline(obj.t);
            r = jsondecode(char(line));
            if isfield(r, 'error') && ~isempty(r.error), error('Studio: %s', r.error); end
            if isfield(r, 'result'), res = r.result; else, res = []; end
        end
        function it = Item(obj, name, type)
            if nargin < 3, type = -1; end
            it = obj.call('Item', [], {name, type});
        end
        function it = AddFrame(obj, name), it = obj.call('AddFrame', [], {name}); end
        function it = AddProgram(obj, name, robot), it = obj.call('AddProgram', [], {name, robot}); end
        function it = AddTarget(obj, name, parent, robot), it = obj.call('AddTarget', [], {name, parent, robot}); end
        function q = Joints(obj, item), q = obj.call('Joints', item); q = q(:)'; end
        function setJoints(obj, item, q), obj.call('setJoints', item, {q(:)'}); end
        function MoveJ(obj, item, target), obj.call('MoveJ', item, {target}); end
        function MoveL(obj, item, target), obj.call('MoveL', item, {target}); end
        function m = Pose(obj, item)
            r = obj.call('Pose', item);
            m = Robolink.decodePose(r);
        end
        function setPose(obj, item, m), obj.call('setPose', item, {m}); end
    end
    methods (Static)
        function v = encode(v)
            % items stay {x_item: id}; 4x4 matrices become {x_pose: rows}; other values as they are
            if isstruct(v) && isfield(v, 'x_item'), v = struct('x_item', v.x_item); return; end
            if isnumeric(v) && isequal(size(v), [4 4]), v = struct('x_pose', v); end
        end
        function m = decodePose(r)
            % {"$pose": rows} decodes to a struct with a 4x4 numeric field x_pose (rows already in place)
            % (jsondecode turns the 4 row arrays into a 4x4 matrix whose rows are those arrays)
            if isstruct(r) && isfield(r, 'x_pose'), m = r.x_pose; else, m = r; end
            if iscell(m), m = cell2mat(cellfun(@(x) x(:)', m(:), 'UniformOutput', false)); end
        end
    end
end
