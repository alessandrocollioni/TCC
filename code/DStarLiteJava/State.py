import Pair as clsPair
class State():
	#Default constructor
	#def State():

	#Equals
	def eq(self, s2):
	
		return ((self.x == s2.x) and (self.y == s2.y));
	

	#Not Equals
	def neq(self, s2):
	
		return ((self.x != s2.x) or (self.y != s2.y));
	

	#Greater than
	def gt(self, s2):
	
		if (self.k.first()-0.00001 > s2.k.first()): 
			return True;
		else:
			if (self.k.first() < s2.k.first()-0.00001): 
				return False;
			return self.k.second() > s2.k.second();
	

	#Less than or equal to
	def lte(self, s2):
	
		if (self.k.first() < s2.k.first()): 
			return True;
		else:
			if (self.k.first() > s2.k.first()):
				return False;
			return self.k.second() < s2.k.second() + 0.00001;
	

	#Less than
	def lt(self, s2):
	
		if (self.k.first() + 0.000001 < s2.k.first()):
			return True;
		else:
			if (self.k.first() - 0.000001 > s2.k.first()):
				return False;
		return self.k.second() < s2.k.second();
	

	#CompareTo Method. self is necessary when self class is used in a priority queue
	def __cmp__(self, that):
	
		#self is a modified version of the gt method
		other = that;
		if (self.k.first()-0.00001 > other.k.first()):
			return 1;
		else:
			if (self.k.first() < other.k.first()-0.00001): return -1;
		if (self.k.second() > other.k.second()): return 1;
		else: 
			if (self.k.second() < other.k.second()): return -1;
		return 0;
	

	#Override the CompareTo function for the HashMap usage
	#@Override
	def __hash__(self):
		return self.x + 34245*self.y;

		

	#@Override 
	def equals(self, aThat) :
	
		#check for self-comparison
		if ( self == aThat ):
			return True;

		#use instanceof instead of getClass here for two reasons
		#1. if need be, it can match any supertype, and not just one class;
		#2. it renders an explict check for "that == null" redundant, since
		#it does the check for null already - "null instanceof [type]" always
		#returns False. (See Effective Java by Joshua Bloch.)
		if ( not(isinstance(aThat, State) )):
			return False;
		#Alternative to the above line :
		#if ( aThat == null or aThat.getClass() != self.getClass() ) return False;

		#cast to native object is now safe
		that = aThat;

		#now a proper field-by-field evaluation can be made
		if (self.x == that.x and self.y == that.y):
			return True;
		return False;

	@classmethod
	def fromState(cls, other):
		"Initialize MyData from a dict's items"
		return cls(other.x, other.y, other.k)
	@classmethod
	def fromNone(cls):
		"Initialize MyData from a dict's items"
		return cls(None, None, None)


	def __init__(self, x, y, k):
		if k== None:
			k=clsPair.Pair(None, None);
        #"Initialize MyData from a sequence"
		self.x = x;
		self.y = y;
		self.k = k;
		