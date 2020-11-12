function pcl_data = pcread_custom(data_name)
f = fopen(data_name,'r');
for i = 1:9
   fgets(f);
end
line = fgets(f);
temp = strsplit(line,' ');
n_pts = str2double(temp{1,2});
line = fgets(f);
data = textscan(f,'%f %f %f %f %f %f\n');
pcl_data.Location = [data{1,1},data{1,2},data{1,3}]';
pcl_data.Color = [];
pcl_data.Normal = [];
pcl_data.Intensity = data{1,4}';
pcl_data.Ring = data{1,5}';
pcl_data.Time = data{1,6}';
pcl_data.Count = n_pts;
pcl_data.XLimits = [];
pcl_data.YLimits = [];
pcl_data.ZLimits = [];

pcl_data.XLimits = [min(pcl_data.Location(:,1)),max(pcl_data.Location(:,1))];
pcl_data.YLimits = [min(pcl_data.Location(:,2)),max(pcl_data.Location(:,2))];
pcl_data.ZLimits = [min(pcl_data.Location(:,3)),max(pcl_data.Location(:,3))];
fclose(f);
end