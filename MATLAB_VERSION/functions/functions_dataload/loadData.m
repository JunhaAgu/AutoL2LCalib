function data = loadData(data_name, dataset_name, mode_environment)
% mode: '2cams1lidar', '2cams2lidars', '2cams3lidars'
time=[];
name_cam0   = cell(0);
name_cam1   = cell(0);

f = fopen([data_name, dataset_name, '/association.txt'], 'r');
fgets(f);

% data mode
n_lidars = -1;
if(strcmp(mode_environment,'2cams1lidar'))
   name_lidar0 = cell(0);
   n_lidars = 1;
   while(1)
      line = fgets(f);
      if(line==-1)
         break;
      else
         temp = strsplit(line,' ');
         time = [time,str2num(temp{1,1})];
         name_cam0 = [name_cam0,[data_name,dataset_name,temp{1,2}]];
         name_cam1 = [name_cam1,[data_name,dataset_name,temp{1,3}]];
         name_lidar0 = [name_lidar0,[data_name,dataset_name,temp{1,4}]];
      end
   end
elseif(strcmp(mode_environment,'2cams2lidars'))
   name_lidar0 = cell(0);
   name_lidar1 = cell(0);
   n_lidars = 2;
   while(1)
      line = fgets(f);
      if(line==-1)
         break;
      else
         temp = strsplit(line,' ');
         time = [time,str2num(temp{1,1})];
         name_cam0 = [name_cam0,[data_name,dataset_name,temp{1,2}]];
         name_cam1 = [name_cam1,[data_name,dataset_name,temp{1,3}]];
         name_lidar0 = [name_lidar0,[data_name,dataset_name,temp{1,4}]];
         name_lidar1 = [name_lidar1,[data_name,dataset_name,temp{1,5}]];
         
      end
   end
elseif(strcmp(mode_environment,'2cams3lidars'))
   name_lidar0 = cell(0);
   name_lidar1 = cell(0);
   name_lidar2 = cell(0);
   n_lidars = 3;
   while(1)
      line = fgets(f);
      if(line==-1)
         break;
      else
         temp = strsplit(line,' ');
         time = [time,str2num(temp{1,1})];
         name_cam0 = [name_cam0,[data_name,dataset_name,temp{1,2}]];
         name_cam1 = [name_cam1,[data_name,dataset_name,temp{1,3}]];
         name_lidar0 = [name_lidar0,[data_name,dataset_name,temp{1,4}]];
         name_lidar1 = [name_lidar1,[data_name,dataset_name,temp{1,5}]];
         name_lidar2 = [name_lidar2,[data_name,dataset_name,temp{1,6}]];
      end
   end
elseif(strcmp(mode_environment,'0cams2lidars'))
    name_lidar0 = cell(0);
    name_lidar1 = cell(0);
    n_lidars = 2;
    while(1)
        line = fgets(f);
        if(line==-1)
            break;
        else
            temp = strsplit(line,' ');
            time = [time,str2num(temp{1,1})];
            name_lidar0 = [name_lidar0,[data_name,dataset_name,temp{1,2}]];
            name_lidar1 = [name_lidar1,[data_name,dataset_name,temp{1,3}]];
            
        end
    end
elseif(strcmp(mode_environment,'0cams1lidars'))
    name_lidar0 = cell(0);
    n_lidars = 1;
    while(1)
        line = fgets(f);
        if(line==-1)
            break;
        else
            temp = strsplit(line,' ');
            time = [time,str2num(temp{1,1})];
            name_lidar0 = [name_lidar0,[data_name,dataset_name,temp{1,2}]];
            
        end
    end
elseif(strcmp(mode_environment,'1cams2lidars'))
    name_lidar0 = cell(0);
    name_lidar1 = cell(0);
    n_lidars = 2;
    while(1)
        line = fgets(f);
        if(line==-1)
            break;
        else
            temp = strsplit(line,' ');
            time = [time,str2num(temp{1,1})];
            name_cam0 = [name_cam0,[data_name,dataset_name,temp{1,2}]];
            name_lidar0 = [name_lidar0,[data_name,dataset_name,temp{1,3}]];
            name_lidar1 = [name_lidar1,[data_name,dataset_name,temp{1,4}]];
            
        end
    end
else
   assert(false,'Error occurs in [loadHCEData]: unknown mode!\n');
end


% load data
n_data = length(time);

if (strcmp(mode_environment,'1cams2lidars'))
    for i = 1:n_data
        img_cam{1,i}.left  = imread(name_cam0{1,i});
    end
elseif (strcmp(mode_environment,'0cams1lidars'))
    
elseif ~(strcmp(mode_environment,'0cams2lidars'))
    img_cam   = cell(1,n_data);
    
    for i = 1:n_data
        img_cam{1,i}.left  = imread(name_cam0{1,i});
        img_cam{1,i}.right = imread(name_cam1{1,i});
    end
end

pcl_lidar = cell(n_lidars,n_data);
if(n_lidars == 1)
   for i = 1:n_data
      pcl_lidar{1,i} = pcread_custom(name_lidar0{1,i});
   end
elseif(n_lidars == 2)
   for i = 1:n_data
      pcl_lidar{1,i} = pcread_custom(name_lidar0{1,i});
      pcl_lidar{2,i} = pcread_custom(name_lidar1{1,i});
   end
elseif(n_lidars == 3)
   for i = 1:n_data
      pcl_lidar{1,i} = pcread_custom(name_lidar0{1,i});
      pcl_lidar{2,i} = pcread_custom(name_lidar1{1,i});
      pcl_lidar{3,i} = pcread_custom(name_lidar2{1,i});
   end
else
   assert(false,'Error occurs in [loadHCEData]: unknown mode!\n');
end



% save data structure
data        = struct('n_data',[],'imgs',[],'pcls',[],'time',[]);
data.n_data = n_data;

if (strcmp(mode_environment,'0cams1lidars'))
    data.pcls   = pcl_lidar;
    data.time   = time;
else
    if ~(strcmp(mode_environment,'0cams2lidars'))
        data.imgs   = img_cam;
    end
    data.pcls   = pcl_lidar;
    data.time   = time;
end