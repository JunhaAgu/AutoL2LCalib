function data = sortRings(data)

% TODO: sorted locations are not needed...!
n_data = data.n_data;
n_lidars = size(data.pcls,1);
for k = 1:n_lidars
   for j = 1:n_data
      data.pcls{k,j}.pcls_rings = cell(1,16);
      index_rings = cell(1,16);
      location_rings = cell(1,16);
      intensity_rings = cell(1,16);
      
      for ch = 1:16
         index_rings{1,ch}     = zeros(1,100000);
         location_rings{1,ch}  = zeros(3,100000);
         intensity_rings{1,ch} = zeros(1,100000);
      end
      
      idxs_ch = ones(1,16);
      for i = 1:data.pcls{k,j}.Count
         num_ring = data.pcls{k,j}.Ring(i) + 1;
         index_rings{1,num_ring}(1,idxs_ch(num_ring)) = i;
         location_rings{1,num_ring}(:,idxs_ch(num_ring)) = data.pcls{k,j}.Location(:,i);
         intensity_rings{1,num_ring}(1,idxs_ch(num_ring)) = data.pcls{k,j}.Intensity(i);
         idxs_ch(num_ring) = idxs_ch(num_ring) + 1;
      end
      
      data.pcls{k,j}.IndexRings = cell(1,16);
      for ch = 1:16
         data.pcls{k,j}.IndexRings{1,ch} = index_rings{1,ch}(1,1:(idxs_ch(ch)-1));
         data.pcls{k,j}.pcls_rings{1,ch}.Location  = location_rings{1,ch}(:,1:(idxs_ch(ch)-1));
         data.pcls{k,j}.pcls_rings{1,ch}.Intensity = intensity_rings{1,ch}(1,1:(idxs_ch(ch)-1));
      end
      fprintf('[%d]-th lidar, [%d]-th measurement...\n',k,j);
   end
end


end