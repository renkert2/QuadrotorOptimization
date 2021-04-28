function dispdisp(o)
% DISPDISP  Verbose display of structures and objects.
%   DISPDISP(Z), where Z is a struct or object, is the same as calling disp(Z),
%   except for each struct or object field or property of Z, the output of that
%   field or property's disp method will be displayed if it normally only
%   requires one line to do so.
% 
%   Use DISPDISP to overload the disp method for your classes whose properties
%   might contain other objects.
%
%   See also struct, classdef, matlab.mixin.CustomDisplay.

% Input checking.
narginchk(1,1);

validType = isobject(o) | isstruct(o);
if ~validType || ~isscalar(o)
    try
        builtin('disp',o);
        return
    catch
        error('Input must be scalar MATLAB object or struct.')
    end
end

S = evalc('builtin(''disp'',o)');

names = fieldnames(o);
a = cellfun('length',names);
fieldWidth = 4 + max(a);
for i = 1:length(names)
    n = names{i};
    try
        val = o.(n);
    catch
        % Value retrieval error.
    end
    
    if isobject(val) || isstruct(val)
        s = evalc('disp(val)');
        
        if isscalar(regexp(s,'[\r\n]+')) % If it's a single (populated) line.
            % Clean up extra line breaks and white space:
            if isstruct(val)
                s = strtrim(regexprep(s,'\s+',' '));
            else
                s = strtrim(s);
            end
            
            leader = sprintf(['%' num2str(fieldWidth) 's: '],n);

            % Replace leader and rest of line up to line break.
            S = regexprep(S,[leader '.*\n??'],[leader s],'dotexceptnewline');
        end
    end
end
disp(S(1:end-1)) % Remove last \n.
end