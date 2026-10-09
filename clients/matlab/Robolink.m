classdef Robolink < handle
    % VerticalBot Studio — MATLAB client (RoboDK-API-like) over the TCP JSON-lines API (port 20501).
    %   rdk = Robolink('127.0.0.1', 20501);
    %   robot = rdk.Item('', 2);                 % ITEM_TYPE_ROBOT
    %   rdk.call('MoveJ', robot, {[0 -90 90 0 90 0]});
    %   pose = rdk.Pose(robot);                  % 4x4 matrix
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
            req = struct('id', obj.seq, 'method', method, 'params', {cellfun(@Robolink.encode, params, 'UniformOutput', false)});
            if ~isempty(target) && isfield(target, 'x_item'), req.target = target.x_item; end
            writeline(obj.t, jsonencode(req));
            line = readline(obj.t);
            r = jsondecode(char(line));
            if isfield(r, 'error') && ~isempty(r.error), error('Studio: %s', r.error); end
            res = r.result;
        end
        function it = Item(obj, name, type)
            if nargin < 3, type = -1; end
            it = obj.call('Item', [], {name, type});
        end
        function it = AddFrame(obj, name), it = obj.call('AddFrame', [], {name}); end
        function it = AddProgram(obj, name, robot), it = obj.call('AddProgram', [], {name, robot}); end
        function it = AddTarget(obj, name, parent, robot), it = obj.call('AddTarget', [], {name, parent, robot}); end
        function q = Joints(obj, item), q = obj.call('Joints', item); end
        function setJoints(obj, item, q), obj.call('setJoints', item, {q}); end
        function m = Pose(obj, item)
            r = obj.call('Pose', item);
            m = reshape([r.x_pose{:}], 4, 4)';
        end
        function setPose(obj, item, m), obj.call('setPose', item, {struct('x_pose', num2cell(m, 2))}); end
    end
    methods (Static)
        function v = encode(v)
            if isstruct(v) && isfield(v, 'x_item'), v = struct('x_item', v.x_item); end
            if isnumeric(v) && isequal(size(v), [4 4]), v = struct('x_pose', num2cell(v, 2)); end
        end
    end
end
% Note: MATLAB's jsonencode maps struct field "x_item"/"x_pose" to "x_item"; the server accepts "$item"/"$pose" —
% use the helper below when a literal "$" key is required:
%   s = jsonencode(struct('a',1)); s = strrep(s, '"x_item"', '"$item"'); s = strrep(s, '"x_pose"', '"$pose"');
