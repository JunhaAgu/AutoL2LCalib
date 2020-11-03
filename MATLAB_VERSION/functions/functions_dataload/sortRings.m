function data = sortRings(data)

n_data = data.n_data;
n_lidars = size(data.pcls,1);
for k = 1:n_lidars
   for j = 1:n_data
      data.pcls{k,j}.pcls_rings = cell(1,16);
      index_rings = cell(1,16);
      location_rings = cell(1,16);
      intensity_rings = cell(1,16);
      for i = 1:data.pcls{k,j}.Count
         num_ring = data.pcls{k,j}.Ring(i) + 1;
         index_rings{1,num_ring} = [index_rings{1,num_ring},i];
         location_rings{1,num_ring} = [location_rings{1,num_ring},data.pcls{k,j}.Location(:,i)];
         intensity_rings{1,num_ring} = [intensity_rings{1,num_ring},data.pcls{k,j}.Intensity(i)];
      end
      data.pcls{k,j}.IndexRings = index_rings;
      
      for i = 1:16
         data.pcls{k,j}.pcls_rings{1,i}.Location = location_rings{1,i};
         data.pcls{k,j}.pcls_rings{1,i}.Intensity = intensity_rings{1,i};
      end
   end
end


end