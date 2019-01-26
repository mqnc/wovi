function c = cost2(joints, jointsBefore)

	k170 = 17.0/18.0;
	b170 = pi*k170/(1.0-2.0*k170);
	a170 = (k170-1.0)/k170 * b170*b170;
	c170 = -a170/b170;

	k120 = 12.0/18.0;
	b120 = pi*k120/(1.0-2.0*k120);
	a120 = (k120-1.0)/k120 * b120*b120;
	c120 = -a120/b120;

	k5 = 0.5/18.0;
	b5 = pi*k5/(1.0-2.0*k5);
	a5 = (k5-1.0)/k5 * b5*b5;
	c5 = -a5/b5;

	fitpart = zeros(13,1);

	if isnan(joints(1))
		c = sqrt(14);
		return;
	end

	% fitness function designed so that
	% f(0)=1, f'(0)=0, f(180)=-1, f'(180)=0, f(120 or 170)=0

	% torsion joints:
	for j=1:2:7
		fitpart(j) = cos( a170/(abs(joints(j))+b170) + c170);
	end
	% hinge joints:
	for j=2:2:6
		fitpart(j) = cos( a120/(abs(joints(j))+b120) + c120);
	end
	% singularities (defined at +-5deg)
	fitpart(8) = -cos( a5/(abs(joints(2))+b5) + c5);
	fitpart(9) = -cos( a5/(abs(joints(4))+b5) + c5);
	fitpart(10) = -cos( a5/(abs(joints(6))+b5) + c5);

	% distance to joints before (should be below 180deg to forbid reconfiguration)
	fitpart(11) = cos( 0.5*(joints(1)-jointsBefore(1)));
	fitpart(12) = cos( 0.5*(joints(3)-jointsBefore(3)));
	fitpart(13) = cos( 0.5*(joints(5)-jointsBefore(5)));
	fitpart(14) = cos( 0.5*(joints(7)-jointsBefore(7)));

	allGreater0 = true;

	for i=1:14
		if fitpart(i) <= 0 
			allGreater0 = false;
		end
	end

	if(allGreater0)
		denom = 0;
		for i=1:14
			denom = denom + 1.0/fitpart(i);
		end
		c = -1.0/denom;
		return;
	else
		radic = 0;
		for i=1:14
			if fitpart(i)<0
				radic = radic + fitpart(i)^2;
			end
		end
		c = sqrt(radic);
	end
end
