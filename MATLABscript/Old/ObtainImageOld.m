clear all; clc;
filename = "./photo8.log";
photo_raw = csvread(filename);

a = './';
b = 'photo';
img = 'photo.png';
sp = strrep(img, '.', '_8.');
full = fullfile(a,b,sp);

bytes_per_pixel = 2;

photo_raw2 = [photo_raw(1, :) photo_raw(3, :) photo_raw(5, :)];
byte_per_line_pre = [photo_raw(2, :) photo_raw(4, :) photo_raw(6, :)];

count = 1;

for i=1:length(byte_per_line_pre)
    
    if  byte_per_line_pre(i) ~= 0
        byte_per_line(count) = byte_per_line_pre(i);
        count = count + 1;
    end
end    

clear photo_raw byte_per_line_pre

count = 1;
numer_line = length(byte_per_line);
max_bytes_per_line = max(byte_per_line);
max_pixels_per_line = max(byte_per_line) / bytes_per_pixel;

if rem(max_pixels_per_line, bytes_per_pixel) ~= 0
    max_pixels_per_line = round(max_pixels_per_line - 1, 0);
end

photo_raw3 = zeros(numer_line, max_bytes_per_line);

 for i=1:numer_line
     
     bytes = byte_per_line(i);
     
     for j=1:bytes
         photo_raw3(i,j) =  photo_raw2(j +bytes*(i - 1));
     end 
 end

R = zeros(numer_line, max_pixels_per_line);
G = zeros(numer_line, max_pixels_per_line);
B = zeros(numer_line, max_pixels_per_line);

 for i=1:numer_line
         
     bytes = byte_per_line(i);
     
     for j=1:bytes_per_pixel:bytes
         
         str = "";
         
         for k = 1:bytes_per_pixel
         
            str = str + dec2hex(photo_raw3(i, j+k-1));
            
         end
         
         number = hex2dec(str);
         
         RR = bitshift(bitand(number, 124), -2);
         GG = bitshift(bitand(number, 57344), -13) + bitshift(bitand(number, 3), 2); 
         BB = bitshift(bitand(number, 7936), -8);
         
         R(i,1+floor(j/bytes_per_pixel)) = RR;
         G(i,1+floor(j/bytes_per_pixel)) = GG;        
         B(i,1+floor(j/bytes_per_pixel)) = BB;
         
         
          
     end
 end
 
 RGB(:,:,1) = R';
 RGB(:,:,2) = G';
 RGB(:,:,3) = B';
 
 RGB = (RGB/255)*3;
 
image(RGB);
 
imwrite(RGB,full);

     