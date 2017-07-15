package us.ihmc.euclid.referenceFrame.tools;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple2D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidFrameAPITestTools
{
   private final static Map<Class<?>, Class<?>> framelessTypesToFrameTypesTable;
   static
   {
      HashMap<Class<?>, Class<?>> modifiableMap = new HashMap<>();
      modifiableMap.put(Tuple2DReadOnly.class, FrameTuple2DReadOnly.class);
      modifiableMap.put(Tuple2DBasics.class, FrameTuple2D.class);
      modifiableMap.put(Point2DReadOnly.class, FramePoint2DReadOnly.class);
      modifiableMap.put(Point2DBasics.class, FramePoint2D.class);
      modifiableMap.put(Vector2DReadOnly.class, FrameVector2DReadOnly.class);
      modifiableMap.put(Vector2DBasics.class, FrameVector2D.class);

      modifiableMap.put(Tuple3DReadOnly.class, FrameTuple3DReadOnly.class);
      modifiableMap.put(Tuple3DBasics.class, FrameTuple3D.class);
      modifiableMap.put(Point3DReadOnly.class, FramePoint3DReadOnly.class);
      modifiableMap.put(Point3DBasics.class, FramePoint3D.class);
      modifiableMap.put(Vector3DReadOnly.class, FrameVector3DReadOnly.class);
      modifiableMap.put(Vector3DBasics.class, FrameVector3D.class);

      framelessTypesToFrameTypesTable = Collections.unmodifiableMap(modifiableMap);
   }

   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameObjects, Class<?> typeWithFramelessObjectsOnly, boolean assertAllCombinations,
                                                        boolean assertOverloadingWithFramelessObjects)
   {
      assertOverloadingWithFrameObjects(typeWithFrameObjects, typeWithFramelessObjectsOnly, assertAllCombinations, assertOverloadingWithFramelessObjects, 1,
                                        m -> true);
   }

   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameObjects, Class<?> typeWithFramelessObjectsOnly, boolean assertAllCombinations,
                                                        boolean assertOverloadingWithFramelessObjects, int minNumberOfGeometryArguments)
   {
      assertOverloadingWithFrameObjects(typeWithFrameObjects, typeWithFramelessObjectsOnly, assertAllCombinations, assertOverloadingWithFramelessObjects,
                                        minNumberOfGeometryArguments, m -> true);
   }

   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameObjects, Class<?> typeWithFramelessObjectsOnly, boolean assertAllCombinations,
                                                        boolean assertOverloadingWithFramelessObjects, int minNumberOfGeometryArguments,
                                                        Map<String, Class<?>[]> methodsToIgnore)
   {
      Predicate<Method> methodFilter = new Predicate<Method>()
      {
         @Override
         public boolean test(Method m)
         {
            for (Entry<String, Class<?>[]> methodToIgnore : methodsToIgnore.entrySet())
            {
               if (m.getName().equals(methodToIgnore.getKey()))
               {
                  if (Arrays.equals(m.getParameterTypes(), methodToIgnore.getValue()))
                     return false;
               }
            }
            return true;
         }
      };

      assertOverloadingWithFrameObjects(typeWithFrameObjects, typeWithFramelessObjectsOnly, assertAllCombinations, assertOverloadingWithFramelessObjects,
                                        minNumberOfGeometryArguments, methodFilter);
   }

   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameObjects, Class<?> typeWithFramelessObjectsOnly, boolean assertAllCombinations,
                                                        boolean assertOverloadingWithFramelessObjects, int minNumberOfGeometryArguments,
                                                        Predicate<Method> framelessMethodFilter)
   {
      // The frame methods are all the methods from 'typeWithFramelessObjectsOnly' that have at least one geometry argument.
      List<Method> framelessMethods = keepOnlyMethodsWithAtLeastOneGeometryArgumentAndNoFrames(typeWithFramelessObjectsOnly.getMethods(),
                                                                                               minNumberOfGeometryArguments);

      for (Method framelessMethod : framelessMethods)
      {
         if (framelessMethodFilter.test(framelessMethod))
         {
            // Creating all the expected combinations
            List<Class<?>[]> expectedMethodSignatures = createExpectedMethodSignaturesWithFrameArgument(framelessMethod, assertAllCombinations);

            for (Class<?>[] expectedMethodSignature : expectedMethodSignatures)
            {
               assertMethodOverloadedWithSpecificSignature(framelessMethod, expectedMethodSignature, typeWithFrameObjects);
            }
         }
      }
   }

   private static void assertMethodOverloadedWithSpecificSignature(Method originalMethod, Class<?>[] overloadingSignature, Class<?> typeToSearchIn)
         throws SecurityException
   {
      try
      {
         Method overloadingMethod = typeToSearchIn.getMethod(originalMethod.getName(), overloadingSignature);
         Class<?> originalReturnType = originalMethod.getReturnType();
         Class<?> overloadingReturnType = overloadingMethod.getReturnType();

         { // Assert the return type is proper
            if (originalReturnType == null && overloadingReturnType != null)
               throw new AssertionError("Inconsistency found in the return type.\nOriginal method: " + getMethodSimpleName(originalMethod)
                     + "\nOverloading method: " + getMethodSimpleName(overloadingMethod));

            if (overloadingReturnType.equals(originalReturnType))
               return;

            if (overloadingReturnType != findCorrespondingFrameType(originalReturnType))
               throw new AssertionError("Unexpected return type: expected: " + findCorrespondingFrameType(originalReturnType).getSimpleName() + ", actual: "
                     + overloadingReturnType.getSimpleName());
         }
      }
      catch (NoSuchMethodException e)
      {
         throw new AssertionError("The original method:\n" + getMethodSimpleName(originalMethod) + "\nis not properly overloaded, expected to find:\n"
               + getMethodSimpleName(originalMethod.getReturnType(), originalMethod.getName(), overloadingSignature));
      }
   }

   private static String getMethodSimpleName(Method method)
   {
      return getMethodSimpleName(method.getReturnType(), method.getName(), method.getParameterTypes());
   }

   private static String getMethodSimpleName(Class<?> returnType, String methodName, Class<?>[] parameterTypes)
   {
      String returnTypeName = returnType == null ? "void" : returnType.getSimpleName();
      return returnTypeName + " " + methodName + "(" + getSimpleNames(parameterTypes) + ")";
   }

   private static String getSimpleNames(Class<?>[] types)
   {
      String ret = Arrays.stream(types).map(t -> t.getSimpleName()).collect(Collectors.toList()).toString();
      return ret.replace("[", "").replace("]", "");
   }

   private static List<Method> keepOnlyMethodsWithAtLeastOneGeometryArgumentAndNoFrames(Method[] methodsToFilter, int minNumberOfFramelessArgument)
   {
      return keepOnlyMethodsWithAtLeastNFramelessArgument(Arrays.asList(methodsToFilter), minNumberOfFramelessArgument);
   }

   private static List<Method> keepOnlyMethodsWithAtLeastNFramelessArgument(List<Method> methodsToFilter, int minNumberOfFramelessArgument)
   {
      List<Method> filteredMethods = new ArrayList<>();

      for (Method method : methodsToFilter)
      {
         int numberOfFramelessArguments = 0;

         for (Class<?> parameterType : method.getParameterTypes())
         {
            if (isFramelessType(parameterType))
            {
               numberOfFramelessArguments++;
               if (numberOfFramelessArguments >= minNumberOfFramelessArgument)
                  break;
            }
         }

         if (numberOfFramelessArguments >= minNumberOfFramelessArgument)
            filteredMethods.add(method);
      }

      return filteredMethods;
   }

   private static List<Class<?>[]> createExpectedMethodSignaturesWithFrameArgument(Method methodWithoutFrameArguments, boolean createAllCombinations)
   {
      Class<?>[] framelessMethodArgumentTypes = methodWithoutFrameArguments.getParameterTypes();
      List<Class<?>[]> expectedMethodSignatures = new ArrayList<>();

      if (!createAllCombinations)
      {
         Class<?>[] combination = new Class<?>[framelessMethodArgumentTypes.length];
         System.arraycopy(framelessMethodArgumentTypes, 0, combination, 0, combination.length);

         for (int k = 0; k < combination.length; k++)
         {
            if (isFramelessType(combination[k]))
               combination[k] = findCorrespondingFrameType(combination[k]);
         }
         expectedMethodSignatures.add(combination);
      }
      else
      {
         int numberOfArgumentsToOverload = (int) Arrays.stream(framelessMethodArgumentTypes).filter(t -> isFramelessType(t)).count();
         int numberOfCombinations = (int) Math.pow(2, numberOfArgumentsToOverload);

         for (int i = 0; i < numberOfCombinations; i++)
         {
            Class<?>[] combination = new Class<?>[framelessMethodArgumentTypes.length];
            System.arraycopy(framelessMethodArgumentTypes, 0, combination, 0, combination.length);
            int currentByte = 0;

            for (int k = 0; k < combination.length; k++)
            {
               if (isFramelessType(combination[k]))
               {
                  int mask = (int) Math.pow(2, currentByte);
                  if ((i & mask) != 0)
                     combination[k] = findCorrespondingFrameType(combination[k]);
                  currentByte++;
               }
            }
            expectedMethodSignatures.add(combination);
         }

         // Remove the original method from the combinations
         for (int combinationIndex = 0; combinationIndex < expectedMethodSignatures.size(); combinationIndex++)
         {
            if (Arrays.equals(framelessMethodArgumentTypes, expectedMethodSignatures.get(combinationIndex)))
            {
               expectedMethodSignatures.remove(combinationIndex);
               break;
            }
         }
      }
      return expectedMethodSignatures;
   }

   private static Class<?> findCorrespondingFrameType(Class<?> framelessType)
   {
      if (!isFramelessType(framelessType))
         throw new IllegalArgumentException("Cannot handle the following type: " + framelessType.getSimpleName());

      Class<?> frameType = null;

      for (Entry<Class<?>, Class<?>> entry : framelessTypesToFrameTypesTable.entrySet())
      {
         if (!entry.getKey().isAssignableFrom(framelessType))
            continue;

         if (frameType == null || frameType.isAssignableFrom(entry.getValue()))
            frameType = entry.getValue();
      }

      if (frameType == null)
         throw new RuntimeException("Could not find the corresponding frame type for: " + framelessType.getSimpleName());

      return frameType;
   }

   private static boolean isFrameType(Class<?> type)
   {
      for (Class<?> frameType : framelessTypesToFrameTypesTable.values())
      {
         if (frameType.isAssignableFrom(type))
            return true;
      }
      return false;
   }

   private static boolean isFramelessType(Class<?> type)
   {
      if (ReferenceFrameHolder.class.isAssignableFrom(type))
         return false;

      for (Class<?> framelessType : framelessTypesToFrameTypesTable.keySet())
      {
         if (framelessType.isAssignableFrom(type))
            return true;
      }
      return false;
   }
}
