function  out = MATReader1C(filename)

load(filename);

im=rescale(im);

[im, masks] = resizeImageandMask(im, masks, [528, 704]);

[im, masks] = resizeImageandMask(im, masks, [528, 704]); %pad to desired size

[im, masks] = augmentImage(im, masks); %random augmentation function

out{1} = im;
out{2} = bbox;
out{3} = label;
out{4} = masks;

end