'''
 * The class <code>Pair</code> models a container for two objects wherein the
 * object order is of no consequence for equality and hashing. An example of
 * using Pair would be as the return type for a method that needs to return two
 * related objects. Another good use is as entries in a Set or keys in a Map
 * when only the unordered combination of two objects is of interest.<p>
 * The term "object" as being a one of a Pair can be loosely interpreted. A
 * Pair may have one or two <code>None</code> entries as values. Both values
 * may also be the same object.<p>
 * Mind that the order of the type parameters T and U is of no importance. A
 * Pair&lt;T, U> can still return <code>True</code> for method <code>equals</code>
 * called with a Pair&lt;U, T> argument.<p>
 * Instances of this class are immutable, but the provided values might not be.
 * This means the consistency of equality checks and the hash code is only as
 * strong as that of the value types.<p>
 *
 * * @author daniel beard
 * http://danielbeard.wordpress.com
 * http://github.com/paintstripper
 *
 * Copyright (C) 2012 Daniel Beard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
'''
class Pair():


    '''
     * Constructs a new <code>Pair&lt;T, U&gt;</code> with T object1 and U object2 as
     * its values. The order of the arguments is of no consequence. One or both of
     * the values may be <code>None</code> and both values may be the same object.
     *
     * @param object1 T to serve as one value.
     * @param object2 U to serve as the other value.
    '''
    def __init__(self, object1,object2) :

        '''
         * One of the two values, for the declared type T.
        '''
        self.object1 = object1;
        '''
         * One of the two values, for the declared type U.
        '''
        self.object2 = object2;
        self.object1None = object1 == None;
        self.object2None = object2 == None;
        self.dualNone = self.object1None and self.object2None;

    

    '''
     * Gets the value of this Pair provided as the first argument in the constructor.
     *
     * @return a value of this Pair.
    '''
    def first(self) :
    

        return self.object1;

    ''

    '''
     * Gets the value of this Pair provided as the second argument in the constructor.
     *
     * @return a value of this Pair.
    '''
    def second(self) :
    

        return self.object2;

    

    '''
     * Sets the value of the first Pair
    '''
    def setFirst(self, object1):
	
	   self.object1 = object1;
	   self.object1None = object1 == None;
	   self.dualNone = self.object1None and self.object2None;
    

    '''
	 * Sets the value of the second pair
	'''
    def setSecond(self, object2):
	
		self.object2 = object2;
		self.object2None = object2 == None;
		self.dualNone = self.object1None and self.object2None;
	


    '''
     * Returns a shallow copy of this Pair. The returned Pair is a new instance
     * created with the same values as this Pair. The values themselves are not
     * cloned.
     *
     * @return a clone of this Pair.
    '''
    #@Override
    def clone(self) :
    

        return Pair(self.object1, self.object2);

    

    '''
     * Indicates whether some other object is "equal" to this one.
     * This Pair is considered equal to the object if and only if
     * <ul>
     * <li>the Object argument is not None,
     * <li>the Object argument has a runtime type Pair or a subclass,
     * </ul>
     * AND
     * <ul>
     * <li>the Object argument refers to this pair
     * <li>OR this pair's values are both None and the other pair's values are both None
     * <li>OR this pair has one None value and the other pair has one None value and
     * the remaining non-None values of both pairs are equal
     * <li>OR both pairs have no None values and have value tuples &lt;v1, v2> of
     * this pair and &lt;o1, o2> of the other pair so that at least one of the
     * following statements is True:
     * <ul>
     * <li>v1 equals o1 and v2 equals o2
     * <li>v1 equals o2 and v2 equals o1
     * </ul>
     * </ul>
     * In any other case (such as when this pair has two None parts but the other
     * only one) this method returns False.<p>
     * The type parameters that were used for the other pair are of no importance.
     * A Pair&lt;T, U> can return <code>True</code> for equality testing with
     * a Pair&lt;T, V> even if V is neither a super- nor subtype of U, should
     * the the value equality checks be positive or the U and V type values
     * are both <code>None</code>. Type erasure for parameter types at compile
     * time means that type checks are delegated to calls of the <code>equals</code>
     * methods on the values themselves.
     *
     * @param obj the reference object with which to compare.
     * @return True if the object is a Pair equal to this one.
    '''
    #@Override
    def equals(self, obj) :
    

        if(obj == None):
            return False;

        if(self == obj):
            return True;

        if(not(isinstanceof(obj,Pair))):
            return False;

        otherPair = obj;

        if(self.dualNone):
            return otherPair.dualNone;

        #After this we're sure at least one part in this is not None

        if(otherPair.dualNone):
            return False;

        #After this we're sure at least one part in obj is not None

        if(self.object1None) :
            if(otherPair.object1None): #Yes: this and other both have non-None part2
                return self.object2.equals(otherPair.object2);
            else:
                if(otherPair.object2None): #Yes: this has non-None part2, other has non-None part1
                    return self.object2.equals(otherPair.object1);
                else: #Remaining case: other has no non-None parts
                    return False;
        else:
            if(self.object2None) :
                if(otherPair.object2None): #Yes: this and other both have non-None part1
                    return self.object1.equals(otherPair.object1);
                else:
                    if(otherPair.object1None): #Yes: this has non-None part1, other has non-None part2
                        return self.object1.equals(otherPair.object2);
                    else: #Remaining case: other has no non-None parts
                        return False;
            else :
                #Transitive and symmetric requirements of equals will make sure
                #checking the following cases are sufficient
                if(self.object1.equals(otherPair.object1)):
                    return self.object2.equals(otherPair.object2);
                else:
                    if(self.object1.equals(otherPair.object2)):
                        return self.object2.equals(otherPair.object1);
                    else:
                        return False;
            

    

    '''
     * Returns a hash code value for the pair. This is calculated as the sum
     * of the hash codes for the two values, wherein a value that is <code>None</code>
     * contributes 0 to the sum. This implementation adheres to the contract for
     * <code>hashCode()</code> as specified for <code>Object()</code>. The returned
     * value hash code consistently remain the same for multiple invocations
     * during an execution of a Java application, unless at least one of the pair
     * values has its hash code changed. That would imply information used for
     * equals in the changed value(s) has also changed, which would carry that
     * change onto this class' <code>equals</code> implementation.
     *
     * @return a hash code for this Pair.
    '''
    #@Override
    def hashCode(self) :
    

        if self.object1None:
            hashCode = 0 
        else: 
            hashCode = self.object1.hashCode();

        if self.object2None:
            hashCode += 0 
        else: 
            hashCode += self.object2.hashCode();

        return hashCode;

    


