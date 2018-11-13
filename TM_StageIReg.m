#!/bin/octave -qf
%printf ("%s", program_name ());
arg_list = argv();
for i = 1:nargin
    folder = arg_list{i};
    folder_n = length(folder);
    imagefiles = glob(["../",folder,"/*.TIF"]);

    Stage1OutputDirMaster = ['../calib_',folder];
    Stage1OutputDirSlave = Stage1OutputDirMaster;

    if exist(Stage1OutputDirMaster, "dir") == 0
        mkdir(Stage1OutputDirMaster);
    end 

    n_files = length(imagefiles);
    for i=1:n_files

        long_name = imagefiles{i,1};
        short_name = long_name(4+folder_n:end);
        
        Img = ReadImageS1(long_name);
        Handbag.Master.FocalLengthX = 4.00;   % mm - along x axis
        Handbag.Master.FocalLengthY = 4.00;   % mm - along y axis
        Handbag.Master.PPX = 2.50202;        % mm - X principal point 
        Handbag.Master.PPY = 1.870268;        % mm - Y principal point 
        Handbag.Master.CCDX = 4.8;            % mm - CCD width (along X)
        Handbag.Master.CCDY = 3.6;            % mm - CCD height (along Y)
        Handbag.Master.nPixelX = 1280;        % pixels - along X axis
        Handbag.Master.nPixelY = 960;         % pixels - along Y axis
        Handbag.Master.FisheyeAffineMat = [1663.093010806,0,0,1663.093010806];
        Handbag.Master.FisheyePoly = [0,1,0.005510503,-0.139223063];
        Handbag.Master.RigRelatives = [0,0,0];
        Handbag.Master.Traslation = [0,0,0];
        if strfind(long_name, "GRE") > 0
            Handbag.Slave.FocalLengthX = 4.00;   % mm - along x axis
            Handbag.Slave.FocalLengthY = 4.00;   % mm - along y axis
            Handbag.Slave.PPX = 2.50202;        % mm - X principal point 
            Handbag.Slave.PPY = 1.870268;        % mm - Y principal point 
            Handbag.Slave.CCDX = 4.8;            % mm - CCD width (along X)
            Handbag.Slave.CCDY = 3.6;            % mm - CCD height (along Y)
            Handbag.Slave.nPixelX = 1280;        % pixels - along X axis
            Handbag.Slave.nPixelY = 960;         % pixels - along Y axis
            Handbag.Slave.FisheyeAffineMat = [1663.093010806,0,0,1663.093010806];
            Handbag.Slave.FisheyePoly = [0,1,0.005510503,-0.139223063];
            Handbag.Slave.RigRelatives = [0,0,0];
            Handbag.Slave.Traslation = [0,0,0];
            SlaveImg = StageIReg(Img,Handbag);
        end
        if strfind(long_name, "RED") > 0
            Handbag.Slave.FocalLengthX = 4.00;   % mm - along x axis
            Handbag.Slave.FocalLengthY = 4.00;   % mm - along y axis
            Handbag.Slave.PPX = 2.264813;        % mm - X principal point 
            Handbag.Slave.PPY = 1.938798;        % mm - Y principal point 
            Handbag.Slave.CCDX = 4.8;            % mm - CCD width (along X)
            Handbag.Slave.CCDY = 3.6;            % mm - CCD height (along Y)
            Handbag.Slave.nPixelX = 1280;        % pixels - along X axis
            Handbag.Slave.nPixelY = 960;         % pixels - along Y axis
            Handbag.Slave.FisheyeAffineMat = [1665.117565771,0,0,1665.117565771];
            Handbag.Slave.FisheyePoly = [0,1,0.003918014,-0.136955888];
            Handbag.Slave.RigRelatives = [-0.208128384622098,-0.0274514057208868,-0.500751997024002];
            Handbag.Slave.Traslation = [0.0,-15.0, 0.0];
            SlaveImg = StageIReg(Img,Handbag);
        end
        if strfind(long_name, "REG") > 0
            Handbag.Slave.FocalLengthX = 4.00;   % mm - along x axis
            Handbag.Slave.FocalLengthY = 4.00;   % mm - along y axis
            Handbag.Slave.PPX = 2.505961;        % mm - X principal point 
            Handbag.Slave.PPY = 1.795225;        % mm - Y principal point 
            Handbag.Slave.CCDX = 4.8;            % mm - CCD width (along X)
            Handbag.Slave.CCDY = 3.6;            % mm - CCD height (along Y)
            Handbag.Slave.nPixelX = 1280;        % pixels - along X axis
            Handbag.Slave.nPixelY = 960;         % pixels - along Y axis
            Handbag.Slave.FisheyeAffineMat = [1670.16616403,0,0,1670.16616403];
            Handbag.Slave.FisheyePoly = [0,1,0.002568249,-0.136560773];
            Handbag.Slave.RigRelatives = [-0.0971708274395733,-0.376325917363549,-0.167151974534878];
            Handbag.Slave.Traslation = [15.0 ,0.0, 0.0];
            SlaveImg = StageIReg(Img,Handbag);
        end
        if strfind(long_name, "NIR") > 0
            Handbag.Slave.FocalLengthX = 4.00;   % mm - along x axis
            Handbag.Slave.FocalLengthY = 4.00;   % mm - along y axis
            Handbag.Slave.PPX = 2.291891;        % mm - X principal point 
            Handbag.Slave.PPY = 1.868807;        % mm - Y principal point 
            Handbag.Slave.CCDX = 4.8;            % mm - CCD width (along X)
            Handbag.Slave.CCDY = 3.6;            % mm - CCD height (along Y)
            Handbag.Slave.nPixelX = 1280;        % pixels - along X axis
            Handbag.Slave.nPixelY = 960;         % pixels - along Y axis
            Handbag.Slave.FisheyeAffineMat = [1669.234554561,0,0,1669.234554561];
            Handbag.Slave.FisheyePoly = [0,1,0.013557718,-0.158449196];
            Handbag.Slave.RigRelatives = [-0.558738957492464,-0.565048898436331,-0.122976488817441];
            Handbag.Slave.Traslation = [15.0 ,-15.0, 0.0];
            SlaveImg = StageIReg(Img,Handbag);
        end
        WriteImageS1(SlaveImg,Stage1OutputDirSlave,short_name,'');
        
        
    end
end
%printf ("\n");


%% ALGORITHM
%folder = "OTROS";
    