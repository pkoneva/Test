#include <fstream>
#include <iostream>
#include <cstdlib>
using namespace std;


class Vector3D {
protected:
	

public:
	double X, Y, Z;
	Vector3D() {
		this->X = 0;
		this->Y = 0;
		this->Z = 0;
	}

	Vector3D(double x, double y, double z)
	{
	    this->X = x;
		this->Y = y;
		this->Z = z;
	}

	void print() const{
		cout << "(" << this->X << ", " << this->Y << ", " << this->Z << ")";
	}
	bool operator==(const Vector3D& that) const {
		return ((this->X == that.X) && (this->Y == that.Y) && (this->Z == that.Z));
	}
};

class Segment3D {
protected:
	Vector3D start;
	Vector3D end;
public:
	Segment3D() {
		this->start = Vector3D(0, 0, 0);
		this->end = Vector3D(0, 0, 0);
	}

	Segment3D(Vector3D S, Vector3D E) {
		this->start = S;
		this->end = E;
	}
	Vector3D Start() {
		return this->start;
	}
	Vector3D End() {
		return this->end;
	}
	void print() const {
		cout << "<" << this->end.X - this->start.X << ", " << this->end.Y - this->start.Y << ", " << this->end.Z - this->start.Z << "> ";
	}

	double Norm(const Segment3D& that) {
		return sqrt(pow(that.end.X - that.start.X, 2) + pow(that.end.Y - that.start.Y, 2) + pow(that.end.Z - that.start.Z, 2));
	}

	double ScalarProduct(const Segment3D& S, const Segment3D& T) {
		return (S.end.X - S.start.X) * (T.end.X - T.start.X) + (S.end.Y - S.start.Y) * (T.end.Y - T.start.Y) + (S.end.Z - S.start.Z) * (T.end.Z - T.start.Z);
	}

	double cos(const Segment3D& S, const Segment3D& T) {
		return ScalarProduct(S, T) / sqrt(Norm(S) * Norm(T));
	}

	bool IsPoint(const Segment3D& that) {
		return Norm(that) == 0;
	}

	bool WhetherBelongToTheSamePlane(const Segment3D& S, const Segment3D& T) {
		return (S.end.X - S.start.X) * (T.start.Y - S.start.Y) * (T.end.Z - S.start.Z) +
			(T.start.X - S.start.X) * (T.end.Y - S.start.Y) * (S.end.Z - S.start.Z) +
			(T.end.X - S.start.X) * (S.end.Y - S.start.Y) * (T.start.Z - S.start.Z) -
			(T.end.X - S.start.X) * (T.start.Y - S.start.Y) * (S.end.Z - S.start.Z) -
			(T.start.X - S.start.X) * (S.end.Y - S.start.Y) * (T.end.Z - S.start.Z) -
			(S.end.X - S.start.X) * (T.end.Y - S.start.Y) * (T.start.Z - S.start.Z) == 0;
	}

	bool IsParallel(const Segment3D& S, const Segment3D& T) {
		if (IsPoint(S) || IsPoint(T)) throw "Points cannot be parallel.";

		double l;
		if (T.end.X - T.start.X != 0) {
			l = (S.end.X - S.start.X) / (T.end.X - T.start.X);
			return ((T.end.Y - T.start.Y == l * (S.end.Y - S.start.Y))
				&& (T.end.Z - T.start.Z == l * (S.end.Z - S.start.Z)));
		}
		else if (S.end.X - S.start.X == 0) {
			if (T.end.Y - T.start.Y != 0) {
				l =  (T.end.Y - T.start.Y)/ (S.end.Y - S.start.Y);
				return (T.end.Z - T.start.Z == l * (S.end.Z - S.start.Z));
			}
			else return (T.end.Y - T.start.Y == 0); 
		}
		else return 0;
	}


	Vector3D Intersect(const Segment3D& S, const Segment3D& T) {
		if (!WhetherBelongToTheSamePlane(S, T))
			throw "No intersections. Skew lines are given.";
		int point_S = IsPoint(S);
		int point_T = IsPoint(T);
		if (point_S & point_T) {
			if (S.start.X == T.start.X) return S.start;
			else throw "No intersections. Two different points are given.";
		}
		else if (point_S || point_T) {
			//if (cos(Segment3D(T.start, S.start), Segment3D(T.end, S.end)) == -1) return (point_S == 1 ? S.start : T.start);
			//if (sqrt(SquaredNorm(Segment3D(T.start, S.start))+ sqrt(SquaredNorm(Segment3D(T.end, S.end))) == sqrt(SquaredNorm(Segment3D(T.end, S.end)))
			if (Norm(Segment3D(T.start, S.start)) + Norm(Segment3D(T.end, S.end)) == Norm(Segment3D(T)) ||
				Norm(Segment3D(T.start, S.start)) + Norm(Segment3D(T.end, S.end)) == Norm(Segment3D(S)))
				return (point_S == 1 ? S.start : T.start);
			else throw "No intersections. A point and a segment are given. The point does not belong to the segment.";
		}
		else {
			if (IsParallel(S, T)) {
				if ((S.start.X - T.end.X) * (T.end.Y - T.start.Y) == (S.start.Y - T.end.Y) * (T.end.X - T.start.X) && //lines are the same
					(S.start.X - T.end.X) * (T.end.Z - T.start.Z) == (S.start.Z - T.end.Z) * (T.end.X - T.start.X) &&
					(S.start.Y - T.end.Y) * (T.end.Z - T.start.Z) == (S.start.Z - T.end.Z) * (T.end.Y - T.start.Y)) {
					if (Norm(S) + Norm(T) > Norm(Segment3D(T.start, S.start)) + Norm(Segment3D(T.end, S.end))) {
						(Norm(Segment3D(T.start, S.end)) > Norm(Segment3D(S.start, T.end)) ?
							throw Segment3D(S.start, T.end) :
							throw Segment3D(T.start, S.end));
					}
					else if (Norm(S) + Norm(T) > Norm(Segment3D(T.start, S.end)) + Norm(Segment3D(S.start, T.end))) {
						(Norm(Segment3D(T.start, S.start)) > Norm(Segment3D(S.end, T.end)) ?
							throw Segment3D(S.end, T.end) :
							throw Segment3D(T.start, S.start));
					}
					else if (Norm(S) + Norm(T) == Norm(Segment3D(T.start, S.end)) + Norm(Segment3D(S.start, T.end))) {
						if (S.start == T.start) throw Segment3D(S.start, T.start);
						if (S.start == T.end) throw Segment3D(S.start, T.end);
						if (S.end == T.end) throw Segment3D(S.end, T.end);
						if (T.start == S.end) throw Segment3D(T.start, S.end);
					}
					else throw "No intersections. Segments belong to the same line, but do not intersect.";
				}
				else throw "No intersections. Segments belong to different parallel lines.";
			}
			else {
				double u, v;
				double d = abs((S.start.X - S.end.X) * (T.start.Y - T.end.Y) - (T.start.X - T.end.X) * (S.start.Y - S.end.Y));
				double d1, d2;
				if (d != 0) {
					d1 = abs((T.end.X - S.end.X) * (T.start.Y - T.end.Y) - (T.start.X - T.end.X) * (T.end.Y - S.end.Y));
					d2 = abs((T.end.Y - S.end.Y) * (S.start.X - S.end.X) - (S.start.Y - S.end.Y) * (T.end.X - S.end.X));
				}
				else {
					d = abs((S.start.Y - S.end.Y) * (T.start.Z - T.end.Z) - (T.start.Y - T.end.Y) * (S.start.Z - S.end.Z));
					//d=(d==0? d :)

					if (d != 0) {
						d1 = abs((T.end.Y - S.end.Y) * (T.start.Z - T.end.Z) - (T.start.Y - T.end.Y) * (T.end.Z - S.end.Z));
						d2 = abs((T.end.Z - S.end.Z) * (S.start.Y - S.end.Y) - (S.start.Z - S.end.Z) * (T.end.Y - S.end.Y));
					}
					else {
						d = abs((S.start.X - S.end.X) * (T.start.Z - T.end.Z) - (T.start.X - T.end.X) * (S.start.Z - S.end.Z));
						d1 = abs((T.end.X - S.end.X) * (T.start.Z - T.end.Z) - (T.start.X - T.end.X) * (T.end.Z - S.end.Z));
						d2 = abs((T.end.Z - S.end.Z) * (S.start.X - S.end.X) - (S.start.Z - S.end.Z) * (T.end.X - S.end.X));
					}
				}
				u = d1 / d;
				v = d2 / d;
				if (!((u <= 1) & (u >= 0) & (v >= 0) & (v <= 1)) )
					throw "Segments belong to intersecting lines, but do not intersect.";
				return Vector3D(u * (S.start.X - S.end.X) + S.end.X, u * (S.start.Y - S.end.Y) + S.end.Y, u * (S.start.Z - S.end.Z) + S.end.Z);
			}
			throw "What ??? I didn't even think about it!";
		}
	}
}; 

int main() {
	Vector3D a;
	/*Vector3D b(2, 3, 4);
	Vector3D c(2, 4, -3);
	Vector3D d(0, 0, -1);
	Vector3D e(0, 0, -2);
	Vector3D f(0, 2, 2);
	Vector3D g(0, 1, 1);
	Vector3D h(0, 0, -3);
	Segment3D t(f, g);
	Segment3D s(g, a);*/
	/*Vector3D p1( 0.5, 0.7,1);
	Vector3D p2(3.9, 2.2,2);
	Vector3D p3( 1.2, 2.4,1);
	Vector3D p4(2.9, 0.9, 2);*/
	Vector3D p1(1, 1, 1);
	Vector3D p2(3, 3, 3);
	Vector3D p3(1, 1, 1);
	Vector3D p4(-3, 2, 0);
	Segment3D t(p1, p2);
	Segment3D s(p3, p4);
	try{

		s.Intersect(s, t).print();
		cout << " is the point of intersection.";
	}
	catch (const char* error_message) {
		cout << error_message;
	}
	catch (Segment3D V) {
		if (V.IsPoint(V)){
			cout << "The segments intersect at the point ";
		    V.Start().print();
		}
		else {
			cout << "The segments intersect along a segment: <";
			V.Start().print();
			cout << "; ";
			V.End().print();
			cout << ">";
		}
	}

	return 0;
}