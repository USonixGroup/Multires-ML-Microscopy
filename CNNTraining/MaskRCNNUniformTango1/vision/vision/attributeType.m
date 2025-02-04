classdef attributeType < uint8
    %attributeType Enumeration of supported attribute types
    %   attributeType creates an enumeration specifying the type of
    %   attribute that can be used to define attributes in the
    %   groundTruth or groundTruthMultiSignal object.
    %
    %   attributeType enumerations:
    %   Numeric  - Attribute's value is a numeric scalar
    %   String   - Attribute's value is a string
    %   Logical  - Attribute's value is logical - logical empty, true or
    %              false
    %   List     - Attribute's value can be a string from a pre-defined
    %              list of strings
    %   None     - Attribute's value is undefined
    %
    %   attributeType methods:
    %   hasValue - Determine if attribute type is Numeric or Logical.
    %
    %
    %   Example: Create attribute definitions table
    %   ---------------------------------------------
    %   % Define attribute names
    %   Name = {'ID'; 'Tag'; 'IsVisible'; 'Color'};
    %
    %   % Define attribute types
    %   Type = attributeType({'Numeric'; 'String'; 'Logical'; 'List'});
    %
    %   % Define attribute values
    %   Value = {int32(1); 'CUSTOM_TAG'; true; {'Red','Green','Blue'}};
    %
    %   % Create label definitions
    %   attribDefs = table(Name, Type, Value)
    %
    %
    % See also labelType.
    
    % Copyright 2017 The MathWorks, Inc.
    
    enumeration
        %Numeric Attribute's value is a numeric scalar
        %   Numeric specifies that attribute's value is a numeric scalar
        Numeric   (0)
        
        %String Attribute's value is a string
        %   String specifies that attribute's value is a nstring
        String        (1)
        
        %Logical Attribute's value is logical
        %   Logical specifies that attribute's value is logical empty,
        %   true or false
        Logical       (2)
        
        %List Attribute's value is defined in a list
        %   List specifies that attribute's value is a string from a
        %   pre-defined list of strings
        List       (3)        
        
        %None Attribute's value is undefined
        %   None specifies that attribute's value is not yet defined       
        None   (4)
    end
    
    methods
        %------------------------------------------------------------------
        function TF = hasValue(this)
            %hasValue Determine if attribute type is Numeric or Logical.
            %   tf = hasValue(this) returns true if attribute type is
            %   Numeric or Logical.
            
            TF = (this==attributeType.Numeric) || (this==attributeType.Logical);
        end
    end
end