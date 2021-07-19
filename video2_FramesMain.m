function video2_FramesMain
    clear all
    close all
    clc    
    
    % call the function that converts a video frames
    readFrames('videos/smartRoboticCartIntroVideo.mp4')
    
 function [ frame ] = readFrames( vid )

    readerobj = VideoReader(vid);
    numFrames = get(readerobj, 'numberOfFrames');    
    %vidFrames = read(readerobj);

    for k = 1 :70: numFrames
        mov(k).cdata = read(readerobj, k);%vidFrames(:,:,:,k);
        mov(k).colormap = [];
        %imshow(mov(k).cdata);
        imname=strcat(int2str(k), '.jpeg');
        filename = ['videos/frames/', imname];
        imwrite(mov(k).cdata, filename);
        %extractComponents(mov(k).cdata);
    end
    fprintf('Done..\n');


% % import the video file
% obj = VideoReader('org.mp4');
% vid = read(obj);
%   
%  % read the total number of frames
% frames = obj.NumberOfFrames;
%   
% % file format of the frames to be saved in
% ST ='.jpg';
%   
% % reading and writing the frames 
% for x = 1 : frames
%   
%     % converting integer to string
%     Sx = num2str(x);
%   
%     % concatenating 2 strings
%     Strc = strcat(Sx, ST);
%     Vid = vid(:, :, :, x);
%     cd frames
%   
%     % exporting the frames
%     imwrite(Vid, Strc);
%     cd ..  
% end