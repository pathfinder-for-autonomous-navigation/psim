function hdf5_helper(filename,sensors,sensors_docs,truth,truth_docs,script_name,seed,condition)
%HDF5_HELPER Writes a HDF5 file with filename and all the data
%   
%   Args:
%       filename(string):
%           File name to save the HDF5 file
%       sensors(struct array):
%           Sensor data from every control cycle
%       sensors_docs(string):
%           Documentation for sensors struct array, include units and frame
%           info here.
%       truth(struct array):
%           The real values to be estimated from every control cycle
%       truth_docs(string):
%           Documentation for sensors struct array, include units and frame
%           info here.
%       script_name(string):
%           Script name
%       seed:
%           seed used
%       condition:
%           condition used
global const
sensorsdetails= struct();
sensorsdetails.Location= '/';%store in root
sensorsdetails.Name= 'sensors';

sdocsattrdet= struct();
sdocsattrdet.AttachedTo= '/sensors';
sdocsattrdet.AttachType= 'dataset';
sdocsattrdet.Name= 'docs';
sdocsattr= sensors_docs;

truthdetails= struct();
truthdetails.Location= '/';%store in root
truthdetails.Name= 'truth';

tdocsattrdet= struct();
tdocsattrdet.AttachedTo= '/truth';
tdocsattrdet.AttachType= 'dataset';
tdocsattrdet.Name= 'docs';
tdocsattr= truth_docs;

timeattrdet= struct();
timeattrdet.AttachedTo= '/';
timeattrdet.AttachType= 'group';
timeattrdet.Name= 'local time made';
timeattr= string(datetime('now'));

scriptattrdet= struct();
scriptattrdet.AttachedTo= '/';
scriptattrdet.AttachType= 'group';
scriptattrdet.Name= 'script file';
scriptattr= script_name;

constattrdet= struct();
constattrdet.AttachedTo= '/';
constattrdet.AttachType= 'group';
constattrdet.Name= 'const';
constattr= const;

seedattrdet= struct();
seedattrdet.AttachedTo= '/';
seedattrdet.AttachType= 'group';
seedattrdet.Name= 'seed';
seedattr= seed;

conditionattrdet= struct();
conditionattrdet.AttachedTo= '/';
conditionattrdet.AttachType= 'group';
conditionattrdet.Name= 'condition';
conditionattr= condition;


hdf5write(filename,sensorsdetails,sensors,truthdetails,truth,...
    timeattrdet,timeattr,...
    scriptattrdet,scriptattr,...
    constattrdet,constattr,...
    seedattrdet,seedattr,...
    conditionattrdet,conditionattr,...
    sdocsattrdet,sdocsattr,...
    tdocsattrdet,tdocsattr);
end

