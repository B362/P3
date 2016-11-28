function y = Friction(u)

dtheta = u(1); B = u(2); grav = u(3);

fric = 0;

if dtheta<0
    fric = -B;
end

if dtheta>0
    fric = B;
end

if dtheta == 0
    fric = -grav;
end

y = fric;