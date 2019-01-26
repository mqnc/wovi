// Derivator by Mirko Kunze
// can be used to analytically and implicitly compute derivatives

#include <math.h>

class dvalue{
public:
	
	dvalue(){};
	dvalue(double value, double derivative){
		x=value;
		dx=derivative;
	}
	
	double x;
	double dx;
};

class dvector{
public:
	dvalue v[3];
	
	dvector(){};
	dvector(dvalue v0, dvalue v1, dvalue v2){
		v[0]=v0;
		v[1]=v1;
		v[2]=v2;
	}	
};

// addition
inline dvalue operator+(const double &s, const dvalue &b){
	dvalue r=b;
	r.x+=s;
	return r;
}

inline dvalue operator+(const dvalue &a, const double &s){
	return s+a;
}

inline dvalue operator+(const dvalue &a, const dvalue &b){
	dvalue r;
	r.x=a.x+b.x;
	r.dx=a.dx+b.dx;
	return r;
}

inline dvector operator+(const dvector &u, const dvector &v){
	dvector r;
	for (int i=0; i<3; i++){
		r.v[i]=u.v[i]+v.v[i];
	}
	return r;
}


// subtraction
inline dvalue operator-(const dvalue &a){
	dvalue r;
	r.x=-a.x;
	r.dx=-a.dx;
	return r;
}

inline dvalue operator-(const double &s, const dvalue &b){
	dvalue r;
	r.x=s-b.x;
	r.dx=-b.dx;
	return r;
}

inline dvalue operator-(const dvalue &a, const double &s){
	dvalue r;
	r.x=a.x-s;
	r.dx=a.dx;
	return r;
}

inline dvalue operator-(const dvalue &a, const dvalue &b){
	dvalue r;
	r.x=a.x-b.x;
	r.dx=a.dx-b.dx;
	return r;
}

inline dvector operator-(const dvector &u, const dvector &v){
	dvector r;
	for (int i=0; i<3; i++){
		r.v[i]=u.v[i]-v.v[i];
	}
	return r;
}


// multiplication
inline dvalue operator*(const double &s, const dvalue &b){
	dvalue r;
	r.x=s*b.x;
	r.dx=s*b.dx;
	return r;		
}

inline dvector operator*(const double &s, const dvector &v){
	dvector r;
	for (int i=0; i<3; i++){
		r.v[i]=s*v.v[i];
	}
	return r;		
}

inline dvalue operator*(const dvalue &a, const dvalue &b){
	dvalue r;
	r.x=a.x*b.x;
	r.dx=a.x*b.dx + a.dx*b.x;
	return r;		
}

inline dvector operator*(const dvalue &a, const dvector &v){
	dvector r;
	for (int i=0; i<3; i++){
		r.v[i]=a.x*v.v[i];
	}
	return r;		
}

inline dvector operator*(const dvector &u, const dvalue &b){
	return b*u;
}

// dot product
inline dvalue operator*(const dvector &u, const dvector &v){
	return u.v[0]*v.v[0] + u.v[1]*v.v[1] + u.v[2]*v.v[2];
}

// cross product
inline dvector cross(const dvector &u, const dvector &v){
	dvector r;
	r.v[0] = u.v[1]*v.v[2] - u.v[2]*v.v[1];
	r.v[1] = u.v[2]*v.v[0] - u.v[0]*v.v[2];
	r.v[2] = u.v[0]*v.v[1] - u.v[1]*v.v[0];
	return r;
}


// division
inline dvalue operator/(const dvalue &a, const dvalue &b){
	dvalue r;
	r.x=a.x/b.x;
	r.dx=(a.dx-r.x*b.dx)/b.x;
	return r;		
}

inline dvector operator/(const dvector &u, const dvalue &b){
	dvector r;
	for (int i=0; i<3; i++){
		r.v[i]=u.v[i]/b;
	}
	return r;		
}


// norm
inline dvalue norm(const dvector &v){
	dvalue r;
	r.x=sqrt(v.v[0].x*v.v[0].x + v.v[1].x*v.v[1].x + v.v[2].x*v.v[2].x);
	r.dx=(v.v[0].x*v.v[0].dx + v.v[1].x*v.v[1].dx + v.v[2].x*v.v[2].dx)/r.x;	
	return r;		
}


// chainrule
inline dvalue chain(const dvalue &a, double (*func)(double), double (*dfunc)(double)){
	dvalue r;
	r.x=func(a.x);
	r.dx=a.dx*dfunc(a.x);
	return r;		
}


// sin
inline dvalue sin(const dvalue &a){
	dvalue r;
	r.x=sin(a.x);
	r.dx=a.dx*cos(a.x);
	return r;		
}

// cos
inline dvalue cos(const dvalue &a){
	dvalue r;
	r.x=cos(a.x);
	r.dx=-a.dx*sin(a.x);
	return r;		
}

// atan2
inline dvalue atan2(const dvalue &y, const dvalue &x){
	dvalue r;
	r.x=atan2(y.x,x.x);
	r.dx=(x.x*y.dx-y.x*x.dx)/(x.x*x.x+y.x*y.x);
	return r;		
}
