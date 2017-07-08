package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.*;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple3D.Tuple3DReadOnlyTest;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class FrameTuple3DReadOnlyTest<T extends FrameTuple3DReadOnly> extends Tuple3DReadOnlyTest<T>
{
   @Override
   public T createEmptyTuple()
   {
      return createTuple(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
   }

   public T createEmptyTuple(ReferenceFrame referenceFrame)
   {
      return createTuple(referenceFrame, 0.0, 0.0, 0.0);
   }

   @Override
   public T createRandomTuple(Random random)
   {
      return createTuple(ReferenceFrame.getWorldFrame(), random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   @Override
   public T createTuple(double x, double y, double z)
   {
      return createTuple(ReferenceFrame.getWorldFrame(), x, y, z);
   }

   public abstract T createTuple(ReferenceFrame referenceFrame, double x, double y, double z);

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = 0.0;

      ReferenceFrame frame1 = ReferenceFrame.getWorldFrame();
      ReferenceFrame frame2 = EuclidFrameRandomTools.generateRandomReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();

      T tuple1 = createTuple(frame1, x, y, z);
      T tuple2 = createTuple(frame1, x, y, z);
      T tuple3 = createTuple(frame2, x, y, z);
      T tuple4 = createTuple(frame2, x, y, z);

      assertTrue(tuple1.epsilonEquals(tuple2, epsilon));
      assertFalse(tuple1.epsilonEquals(tuple3, epsilon));
      assertFalse(tuple3.epsilonEquals(tuple2, epsilon));
      assertTrue(tuple3.epsilonEquals(tuple4, epsilon));
   }

   @Test
   public void testEquals() throws Exception
   {
      super.testEquals();

      Random random = new Random(621541L);

      ReferenceFrame frame1 = ReferenceFrame.getWorldFrame();
      ReferenceFrame frame2 = EuclidFrameRandomTools.generateRandomReferenceFrame(random);

      double x = random.nextDouble();
      double y = random.nextDouble();
      double z = random.nextDouble();

      T tuple1 = createTuple(frame1, x, y, z);
      T tuple2 = createTuple(frame1, x, y, z);
      T tuple3 = createTuple(frame2, x, y, z);
      T tuple4 = createTuple(frame2, x, y, z);

      assertTrue(tuple1.equals(tuple2));
      assertFalse(tuple1.equals(tuple3));
      assertFalse(tuple3.equals(tuple2));
      assertTrue(tuple3.equals(tuple4));
   }

   @Test
   public void testOverloading() throws Exception
   {
      assertSuperMethodsAreOverloaded(FrameTuple3DReadOnly.class, Tuple3DReadOnly.class, FrameTuple3DReadOnly.class, Tuple3DReadOnly.class);
   }

   /**
    * The main purpose of this method is to ensure that FrameObject, extending Object, is
    * overloading all the original method such as "set(Object)" with "set(FrameObject)".
    * 
    * @param overloadingParameterType the type with which the typeToTest should be overloading super
    *           methods.
    * @param originalParameterType the type look for for in the super type.
    * @param typeToTest the type (class or interface) to test the API of.
    * @param superType the super type of the tested type.
    */
   public static <A extends B, B, C extends D, D> void assertSuperMethodsAreOverloaded(Class<A> overloadingParameterType, Class<B> originalParameterType,
                                                                                       Class<C> typeToTest, Class<D> superType)
   {
      assertTrue(typeToTest.getSimpleName() + " is not an extension/implementation of " + superType.getSimpleName(), superType.isAssignableFrom(typeToTest));

      Method[] methodsToTest = typeToTest.getMethods();
      Method[] superMethods = superType.getMethods();

      // The idea is to first filter out the methods that we are not interested in, making debugging much simpler.
      List<Method> filteredSuperMethods = new ArrayList<>(Arrays.asList(superMethods));
      List<Method> filteredMethodsToTest = new ArrayList<>();

      // Let's first remove the methods that come from the super type
      for (Method method : methodsToTest)
      {
         boolean isASuperMethod = filteredSuperMethods.stream().filter(superMethod -> superMethod.getName().equals(method.getName()))
                                                      .filter(superMethod -> Arrays.equals(superMethod.getParameterTypes(), method.getParameterTypes()))
                                                      .findAny().isPresent();

         if (!isASuperMethod)
            filteredMethodsToTest.add(method);
      }

      filteredMethodsToTest = filteredMethodsToTest.stream().filter(m -> m.getParameterTypes().length > 0).collect(Collectors.toList());

      // Then remove all the methods in the super type that do not have the argument that we want to check.
      for (int i = 0; i < filteredSuperMethods.size();)
      {
         Method method = filteredSuperMethods.get(i);
         boolean isMethodOfInterest = false;

         for (Class<?> parameterType : method.getParameterTypes())
         {
            if (originalParameterType.isAssignableFrom(parameterType))
            {
               isMethodOfInterest = true;
               break;
            }
         }

         if (!isMethodOfInterest)
            filteredSuperMethods.remove(i);
         else
            i++;
      }

      for (int j = 0; j < filteredSuperMethods.size(); j++)
      {
         Method superMethod = filteredSuperMethods.get(j);
         List<Method> overloadCandidates = new ArrayList<>(filteredMethodsToTest);

         // Let's remove the methods with different name
         for (int i = 0; i < overloadCandidates.size();)
         {
            Method candidate = overloadCandidates.get(i);
            if (!candidate.getName().equals(superMethod.getName()))
               overloadCandidates.remove(i);
            else
               i++;
         }

         // Let's remove the methods with different number of arguments
         for (int i = 0; i < overloadCandidates.size();)
         {
            Method candidate = overloadCandidates.get(i);
            if (candidate.getParameterTypes().length != superMethod.getParameterTypes().length)
               overloadCandidates.remove(i);
            else
               i++;
         }

         // Finally let's check each argument one at a time.
         Class<?>[] superParameterTypes = superMethod.getParameterTypes();

         // Creating all the expected combinations
         int numberOfArgumentsToOverload = (int) Arrays.stream(superParameterTypes).filter(t -> originalParameterType.isAssignableFrom(t)).count();
         int numberOfCombinations = (int) Math.pow(2, numberOfArgumentsToOverload);

         List<Class<?>[]> expectedParameterTypes = new ArrayList<>();

         for (int i = 0; i < numberOfCombinations; i++)
         {
            Class<?>[] combination = new Class<?>[superParameterTypes.length];
            System.arraycopy(superParameterTypes, 0, combination, 0, combination.length);
            int currentByte = 0;

            for (int k = 0; k < combination.length; k++)
            {
               if (originalParameterType.isAssignableFrom(combination[k]))
               {
                  int mask = (int) Math.pow(2, currentByte);
                  if ((i & mask) != 0)
                     combination[k] = overloadingParameterType;
                  currentByte++;
               }
            }
            expectedParameterTypes.add(combination);
         }

         // Remove the super method from the combinations
         for (int combinationIndex = 0; combinationIndex < expectedParameterTypes.size(); combinationIndex++)
         {
            if (Arrays.equals(superParameterTypes, expectedParameterTypes.get(combinationIndex)))
            {
               expectedParameterTypes.remove(combinationIndex);
               break;
            }
         }

         // Now look for each combination
         for (int combinationIndex = 0; combinationIndex < expectedParameterTypes.size(); combinationIndex++)
         {
            boolean hasCombinationBeenFound = false;
            Class<?>[] combination = expectedParameterTypes.get(combinationIndex);

            // Find the combination in the candidate list
            for (int i = 0; i < overloadCandidates.size(); i++)
            {
               Method candidate = overloadCandidates.get(i);
               Class<?>[] candidateParameterTypes = candidate.getParameterTypes();
               boolean candidateMatchesCombination = true;

               for (int paramIndex = 0; paramIndex < combination.length; paramIndex++)
               {
                  Class<?> candidateParamType = candidateParameterTypes[paramIndex];
                  Class<?> combinationParamType = combination[paramIndex];
                  Class<?> superParamType = superParameterTypes[paramIndex];

                  if (combinationParamType == superParamType)
                  {
                     if (!combinationParamType.equals(candidateParamType))
                     {
                        candidateMatchesCombination = false;
                        break;
                     }
                  }
                  else if (!combinationParamType.isAssignableFrom(candidateParamType))
                  {
                     candidateMatchesCombination = false;
                     break;
                  }
               }

               if (candidateMatchesCombination)
               {
                  hasCombinationBeenFound = true;
                  break;
               }
            }

            if (!hasCombinationBeenFound)
            {
               String message = "The method: \"" + superMethod.getName() + "(" + methodNiceStringOfArguments(superParameterTypes)
                     + ")\" is not properly overloaded.\nExpected to find: \"";
               message += typeToTest.getSimpleName() + "." + superMethod.getName() + "(" + methodNiceStringOfArguments(combination) + ")\"";
               throw new AssertionError(message);
            }
         }
      }
   }

   public static String methodNiceStringOfArguments(Class<?>[] types)
   {
      String ret = Arrays.stream(types).map(t -> t.getSimpleName()).collect(Collectors.toList()).toString();
      return ret.replace("[", "").replace("]", "");
   }

   public static String expectedMethodNiceStringOfArguments(Class<?>[] types, Class<?> superType, Class<?> expectedType)
   {
      Class<?>[] expectedTypes = new Class<?>[types.length];
      System.arraycopy(types, 0, expectedTypes, 0, types.length);

      for (int i = 0; i < expectedTypes.length; i++)
      {
         if (superType.isAssignableFrom(expectedTypes[i]))
            expectedTypes[i] = expectedType;
      }
      return methodNiceStringOfArguments(expectedTypes);
   }
}
