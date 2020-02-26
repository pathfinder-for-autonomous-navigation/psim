function czml2html(czml,filename,show)
%CZML2HTML takes a czml cell array and saves it a html file
%uses template.html it also opens the html file in the default browser.
%   show(optional logical arg): if false, don't open the html in the browser

% Started by Nathan Zimmerberg on Feb 16, 2019
% Authors: Nathan Zimmerberg (nhz2@cornell.edu)
% Latest Revision: Feb 16, 2019
% Pathfinder for Autonomous Navigation
% Space Systems Design Studio
% Cornell University
if nargin<=2
    show= true;
end
myjson=string(jsonencode(czml));
tempfile=fopen("template.html");
template= string(fread(tempfile,'*char')');
fclose(tempfile);
fullhtml = strrep(template,"PSIM_CZML",myjson);
file = fopen(filename,'w');
fprintf(file, fullhtml);
fclose(file);


if show
    web(filename,'-browser')
end
end

