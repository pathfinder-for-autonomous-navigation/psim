function czml2html(czml,filename,show)
%CZML_STRUCT2HTMLSTRING takes a czml cell array and saves it a html file
%   Detailed explanation goes here
if nargin<=2
    show= true;
end
myjson=string(jsonencode(czml));
tempfile=fopen("template.html");
template= string(fread(tempfile,'*char')');
fclose(tempfile);
fullhtml = strrep(template,"PSIM_CZML",myjson);
file = fopen(filename,'wt');
fprintf(file, fullhtml);
fclose(file);


if show
    web(filename,'-browser')
end
end

