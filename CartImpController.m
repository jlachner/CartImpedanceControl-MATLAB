classdef CartImpController < handle
    % A class that implements a Cartesian Impedance Controller
    %
    % Please cite:
    % @phdthesis{lachner2022geometric,
    %   title={A geometric approach to robotic manipulation in physical human-robot interaction},
    %   author={Lachner, Johannes},
    %   school={University of Twente},
    %   year={2022}
    % }

    properties

        Name = 'CartesianImpedanceController';

        Type;       % Two types implemented: translational and rotational

        p           % Position on robot body (3x1)
        p_des         % Desired position (3x1)
        del_p       % Delta between positions (3x1)

        euler_des     % Euler angles of desired orientation wrt. base frame (1x3)
        R_0_b       % Rotation matrix of current orientation wrt. base frame (3x3)
        epsilon     % Axis of desired orientation wrt. body-fixed frame  (3x1)
        eta         % Rotation angle (scalar)

        K_p         % Linear stiffness (3x3)
        K_e         % Rotational stiffness (3x3)

    end

    methods
        function obj = CartImpController( type )
            % Declaration of desired controller, either "translational" or
            % "rotational" Cartesian Impedance Controller
            assert( strcmp( type, 'translational' ) || strcmp( type, 'rotational' ), ...
                'Choose either "translational" or "rotational."' )
            obj.Type = type;

        end


        function obj = setKinematics( obj, x )
            % The controller needs either the position or the Rotation 
            % matrix of the body-fixed coordinate frame
            assert( isequal( size( x ), [ 3, 1 ] ) || isequal( size( x ), [ 3, 3 ] )  , ...
                'An input to this function must be a 3-by-1 or 3-by-3' )

            switch obj.Type
                case 'translational'
                    obj.p = x;
                case 'rotational'
                    obj.R_0_b = x;
            end

        end


        function obj = setStiffness( obj, K )
            % This method is to set the desired stiffness, expressed 
            % in body-fixed coordinates
            assert( isequal( size( K ), [ 3, 3 ] ), ...
                'An input to this function must be a 3-by-3' )

            switch obj.Type
                case 'translational'
                    obj.K_p = K;
                case 'rotational'
                    obj.K_e = K;
            end

        end


        function obj = setZFT( obj, x_0 )
            % Either the desired position or desired Euler angles ("ZYX") 
            % have to be defined, expressed in base coordinates
            assert( isequal( size( x_0 ), [ 3, 1 ] ) || isequal( size( x_0 ), [ 1, 3 ] ) , ...
                'An input to this function must be a 3-by-1 or 1-by-3' )

            switch obj.Type
                case 'translational'
                    obj.p_des = x_0;
                case 'rotational'
                    obj.euler_des = x_0';
            end

        end


        function F = getWrench( obj )
            % To calculate the wrench that minimized the elastic potential
            % Note: ZFT, Stiffness, and Kinematics have to be set first!
            switch obj.Type
                case 'translational'
                    assert( isequal( size( obj.p_des ), [ 3, 1 ] ) , ...
                        'ZFT not yet defined!' )
                    assert( isequal( size( obj.K_p ), [ 3, 3 ] ) , ...
                        'Stiffness not yet defined!' )
                    assert( isequal( size( obj.p ), [ 3, 1 ] ) , ...
                        'Kinematics not yet defined!' )

                    obj.del_p = obj.p_des - obj.p;
                    del_p_mat = vec_to_so3( obj.del_p );
                    F = [ - obj.K_p * obj.del_p; ...
                        del_p_mat * obj.K_p * obj.del_p ];

                case 'rotational'
                    assert( isequal( size( obj.euler_des ), [ 1, 3 ] ) , ...
                        'ZFT not yet defined!' )
                    assert( isequal( size( obj.K_e ), [ 3, 3 ] ) , ...
                        'Stiffness not yet defined!' )
                    assert( isequal( size( obj.R_0_b ), [ 3, 3 ] ) , ...
                        'Kinematics not yet defined!' )

                    R_0_des = eul2rotm( obj.euler_des );
                    R_b_des = obj.R_0_b' * R_0_des;
                    quat_b_des =rotm2quat( R_b_des );
                    obj.eta = quat_b_des( 1 );
                    obj.epsilon = quat_b_des( 2:4 );
                    E = obj.eta * eye(3) - vec_to_so3( obj.epsilon );
                    F = [ zeros( 3, 1 ); ...
                        2 * E' * obj.K_e * obj.epsilon' ];

            end

        end

    end
end

