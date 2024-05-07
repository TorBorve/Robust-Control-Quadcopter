%% Clear command window
clc;


%% Choose matrix to convert
M = tau_lin
M = string(M);


%% Convert

% Get matrix dimensions
m = size(M, 1);
n = size(M, 2);

% Create first line
s = sprintf('  \\begin{bmatrix}\n  ');

% Add matrix content
max_expr_length = max(strlength(M), [], "all");
for k = 1:m
    for l = 1:n
        s = sprintf('%s %s', s, M(k, l)); % print 3 decimal places, align to 6 characters
        if l < n
            s = sprintf('%s &', s);
        end
    end
    if k < m
        s = sprintf('%s \\cr', s);
    end
    s = sprintf('%s\n  ', s);
end

% Add last line
s = sprintf('%s\\end{bmatrix}\n', s);

% Print the result
disp(s);
