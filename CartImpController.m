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

        dq          % Joint velocity  (nx1)

        p           % Position on robot body (3x1)
        p_des       % Desired position (3x1)
        del_p       % Delta between positions (3x1)

        J_p         % Linear part of the Jacobian (3xn)
        J_e         % Rotational part of the Jacobian (3xn)
        M           % Mass matrix

        euler_des     % Euler angles of desired orientation wrt. base frame (1x3)
        R_0_b       % Rotation matrix of current orientation wrt. base frame (3x3)
        epsilon     % Axis of desired orientation wrt. body-fixed frame  (3x1)
        eta         % Rotation angle (scalar)

        K_p         % Linear stiffness matrix (3x3)
        K_e         % Rotational stiffness matrix (3x3)

        B_p         % Linear damping matrix (3x3)
        B_e         % Rotational damping matrix (3x3)

        Zeta_p      % Linear damping ration
        Zeta_e      % Rotational damping ration

    end

    methods
        function obj = CartImpController( type )
            % Declaration of desired controller, either "translational" or
            % "rotational" Cartesian Impedance Controller
            assert( strcmp( type, 'translational' ) || strcmp( type, 'rotational' ), ...
                'Choose either "translational" or "rotational."' )
            obj.Type = type;

        end


        function obj = setKinematics( obj, dq, x, J, M )
            % The controller needs either the position or the Rotation
            % matrix of the body-fixed coordinate frame
            assert( isequal( size( x ), [ 3, 1 ] ) || isequal( size( x ), [ 3, 3 ] )  , ...
                'An input to this function must be a 3-by-1 or 3-by-3' )

            switch obj.Type
                case 'translational'
                    obj.p = x;
                    obj.J_p = J;
                case 'rotational'
                    obj.R_0_b = x;
                    obj.J_e = J;
            end

            obj.dq = dq;
            obj.M = M;

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
            % Specify Zero-Force-Trajectory.
            % Either the desired position or the desired Euler angles ("ZYX")
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


        function F_elas = getElasticWrench( obj )
            % To calculate the wrench that minimized the elastic potential
            % The wrench is expressed wrt. tool coordinates
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
                    F_elas = [ - obj.K_p * obj.del_p; ...
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
                    F_elas = [ zeros( 3, 1 ); ...
                        2 * E' * obj.K_e * obj.epsilon' ];

            end

        end


        function obj = setDampingRatio( obj, Xi )
            % This method is to set the desired stiffness, expressed
            % in body-fixed coordinates
            assert( isequal( size( K ), [ 3, 3 ] ), ...
                'An input to this function must be a 3-by-3' )

            switch obj.Type
                case 'translational'
                    obj.Zeta_p = Xi;
                case 'rotational'
                    obj.Zeta_e = Xi;
            end

        end


        function Dx_b = getDampingMatrix( obj )
            % Desired damping in tool coordinates
            % Damping based on factorization design

            Ad_H_0_b = eye(6);
            Ad_H_0_b(1:3,1:3) = obj.R_0_b;
            Ad_H_0_b(4:6,4:6) = obj.R_0_b;
            Ad_H_b_0 = eye(6);
            Ad_H_b_0(1:3,1:3) = obj.R_0_b';
            Ad_H_b_0(4:6,4:6) = obj.R_0_b';

            % Calculate desired damping in tool coordinates
            [ lam, lam_sqrt ] = getLambdaLeastSquares( obj );
            A = Ad_H_b_0 * lam_sqrt * Ad_H_0_b;
            Kd1 = sqrt(obj.K_p);
            D_zeta = obj.Zeta_p * eye(6);
            Dx_b = A * D_zeta * Kd1 + Kd1 * D_zeta * A;
            %             Dx_0 = Ad_H_0_b * Dx_b *  Ad_H_b_0;

        end


        function F_damp = getDampingWrench( obj )
            B = getDampingMatrix( obj );

            switch obj.Type
                case 'translational'
                    [ m , n ] = size( obj.J_p );
                    J = [ obj.J_p; zeros( m, n ) ];
                    twist = J * obj.dq;
                case 'rotational'
                    [ m , n ] = size( obj.J_e );
                    J = [ zeros( m, n ); obj.J_e ];
                    twist = J * obj.dq;
            end

            F_damp = B * twist;

        end


        function [ Lambda, Lambda_sqrt ] = getLambdaLeastSquares( obj )
            % Lambda with respect to base coordinates
            M_inv = obj.M \ eye( size( obj.M ) );

            switch obj.Type
                case 'translational'
                    [ m , n ] = size( obj.J_p );
                    J = [ obj.J_p; zeros( m, n ) ];
                    Lambda_inv = J * M_inv * J';
                case 'rotational'
                    [ m , n ] = size( obj.J_e );
                    J = [ zeros( m, n ); obj.J_e ];
                    Lambda_inv = J * M_inv * J';
            end

            [ u, s, ~ ] = svd(Lambda_inv);                    % least-square solutions for damping design
            s_diag = diag(s);
            min_sing_val = 0.002;
            if( min( s_diag ) < min_sing_val )
                for i = 1 : 1 : length( s_diag )
                    if s_diag( i ) < min_sing_val
                        s_diag( i ) = min_sing_val;
                    end
                end
            end
            s_inv = 1 ./ s_diag;
            s_inv_sqrt = sqrt( s_inv );
            Lambda = u * diag( s_inv ) * u';                % you can use u instead of v because the matrix is p.s.d.
            Lambda_sqrt = u * diag( s_inv_sqrt ) * u';

        end

    end
end

