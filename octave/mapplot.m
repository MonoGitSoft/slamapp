clear all;
load map.m
mapN = size(mappose);
mapN = mapN(1);
for i = [1:2: (mapN - 1)]
  asd = mappose(i:i+1,:); 
  plot(asd(1),asd(2),"linewidth",4);
end