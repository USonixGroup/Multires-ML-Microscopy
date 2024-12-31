classdef BagOfFeaturesDBoWBuildable < coder.ExternalDependency %#codegen
    %

    % Copyright 2024 The MathWorks, Inc.
    
    properties (Constant, Access = private)
        APIHeaderFile = "bagOfVisualWordsDBoW_api.hpp";
    end

    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'bagOfFeaturesDBoW';
        end
        
        function b = isSupportedContext(~)
            b = true; % supports non-host target
        end
        
        function updateBuildInfo(buildInfo, context)
            
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include'), ...
                fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','ocv','include'), ...
                fullfile(matlabroot,'toolbox', ...
                'images','opencv','opencvcg', 'include')} );
            
            srcPaths = {fullfile(matlabroot,'toolbox', ...
                            'vision','builtins','src','vision'), ...
                            fullfile(matlabroot,'toolbox', ...
                            'vision','builtins','src','ocv'), ...
                            fullfile(matlabroot,'toolbox', ...
                            'vision','builtins','src','vision')};
            buildInfo.addSourceFiles({'bagOfVisualWordsDBoW_api.cpp', ...
                'cgCommon.cpp', ...
                'BagOfVisualWordsDBoWImpl.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                                        'bagOfVisualWordsDBoW_api.hpp', ...
                                        'cgCommon.hpp', ...
                                        'TemplatedVocabularySerializable.hpp'});

            % Add DBoW header files
            dbow2Path = fullfile(matlabroot,'toolbox','vision',...
                'builtins','src','thirdparty','DBoW2');
            dbow2HeaderPath = fullfile(dbow2Path,'include','DBoW2');
            dbow2HeaderFiles = dir(fullfile(dbow2HeaderPath,'*.h'));
            buildInfo.addIncludePaths(dbow2HeaderPath);
            buildInfo.addIncludeFiles({dbow2HeaderFiles.name});
            
            vision.internal.buildable.portableOpenCVBuildInfo(buildInfo, context, ...
                'bagOfFeaturesDBoW');

            % Add DBoW binary file
            arch            = computer('arch');
            pathBinArch     = fullfile(matlabroot, 'bin', arch, filesep);

            switch arch
                case {'win64'}
                    linkFiles = 'DBoW2.lib';
                case {'glnxa64'}
                    linkFiles = 'libDBoW2.so';
                otherwise
                    linkFiles = 'libDBoW2.dylib';
            end
            group = 'BlockModules';
            buildInfo.addLinkObjects(linkFiles, pathBinArch, '', ...
                        true, true, group);
        end
    end

    properties (Access = protected)
        ObjInternal
    end

    methods
        function obj = BagOfFeaturesDBoWBuildable()
            %isSupportedContext
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            obj.ObjInternal = coder.opaquePtr('void', coder.internal.null);
        end

        function [depthLevel, branchingFactor, normalization, obj] = createBagFromFile(obj, bagFile)
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            depthLevel = int32(0);
            branchingFactor = int32(0);
            normalization = 'L1';
            % To generate C/C++ strings from MATLAB character row vectors, 
            % the MATLAB character row vector must be null-terminated (end with zero, 0)
            bagFile0 = [bagFile 0];
            obj.ObjInternal = coder.ceval("DBoW_Bag_createBagFromFile", ...
                coder.ref(bagFile0), coder.wref(depthLevel), coder.wref(branchingFactor), coder.ref(normalization));
        end
        
        function serializedBag = getSerializedBag(obj)
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            serializedBag = '';
            coder.ceval("DBoW_Bag_getSerializedBag", obj.ObjInternal, coder.ref(serializedBag));
        end
    end
end
