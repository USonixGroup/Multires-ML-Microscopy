function out = reorderDatastoreOutputs(in, order)
% reorderDatastoreOutputs is to be used as a transform function to reorder
% the outputs of a datastore based on the specified 'order'.

for i = 1:numel(order)
 out{i} = in{order(i)}; %#ok<AGROW> 
end