package us.ihmc.euclid.referenceFrame.tools;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.junit.Assert;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple2D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.RigidBodyTransform;
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

   private final static Map<Class<?>, FrameTypeBuilder> frameTypeBuilders;
   static
   {
      HashMap<Class<?>, FrameTypeBuilder> modifiableMap = new HashMap<>();
      modifiableMap.put(FrameTuple2DReadOnly.class, FramePoint2D::new);
      modifiableMap.put(FrameTuple2D.class, FramePoint2D::new);
      modifiableMap.put(FramePoint2DReadOnly.class, FramePoint2D::new);
      modifiableMap.put(FramePoint2D.class, FramePoint2D::new);
      modifiableMap.put(FrameVector2DReadOnly.class, FrameVector2D::new);
      modifiableMap.put(FrameVector2D.class, FrameVector2D::new);

      modifiableMap.put(FrameTuple3DReadOnly.class, FramePoint3D::new);
      modifiableMap.put(FrameTuple3D.class, FramePoint3D::new);
      modifiableMap.put(FramePoint3DReadOnly.class, FramePoint3D::new);
      modifiableMap.put(FramePoint3D.class, FramePoint3D::new);
      modifiableMap.put(FrameVector3DReadOnly.class, FrameVector3D::new);
      modifiableMap.put(FrameVector3D.class, FrameVector3D::new);

      frameTypeBuilders = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Map<Class<?>, GenericTypeBuilder> genericTypeBuilders;
   static
   {
      HashMap<Class<?>, GenericTypeBuilder> modifiableMap = new HashMap<>();
      modifiableMap.put(AxisAngleBasics.class, AxisAngle::new);

      genericTypeBuilders = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Set<Class<?>> frameReadOnlyTypes;
   static
   {
      Set<Class<?>> modifiableSet = new HashSet<>();
      modifiableSet.add(FrameTuple2DReadOnly.class);
      modifiableSet.add(FramePoint2DReadOnly.class);
      modifiableSet.add(FrameVector2DReadOnly.class);
      modifiableSet.add(FrameTuple3DReadOnly.class);
      modifiableSet.add(FramePoint3DReadOnly.class);
      modifiableSet.add(FrameVector3DReadOnly.class);

      frameReadOnlyTypes = Collections.unmodifiableSet(modifiableSet);
   }

   private final static Set<Class<?>> frameMutableTypes;
   static
   {
      Set<Class<?>> modifiableSet = new HashSet<>();
      modifiableSet.add(FrameTuple2D.class);
      modifiableSet.add(FramePoint2D.class);
      modifiableSet.add(FrameVector2D.class);
      modifiableSet.add(FrameTuple3D.class);
      modifiableSet.add(FramePoint3D.class);
      modifiableSet.add(FrameVector3D.class);

      frameMutableTypes = Collections.unmodifiableSet(modifiableSet);
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
         boolean assertOverloadingWithFramelessObjects, int minNumberOfGeometryArguments, Map<String, Class<?>[]> methodsToIgnore)
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
         boolean assertOverloadingWithFramelessObjects, int minNumberOfGeometryArguments, Predicate<Method> framelessMethodFilter)
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
               assertMethodOverloadedWithSpecificSignature(typeWithFrameObjects, typeWithFramelessObjectsOnly, framelessMethod, expectedMethodSignature,
                     typeWithFrameObjects);
            }
         }
      }
   }

   public static void assertStaticMethodsCheckReferenceFrame(Class<?> typeHoldingStaticMethodsToTest, boolean shouldThrowExceptionForReadOnlies,
         boolean shouldThrowExceptionForMutables, boolean shouldChangeFrameOfMutables) throws Throwable
   {
      assertStaticMethodsCheckReferenceFrame(typeHoldingStaticMethodsToTest, shouldThrowExceptionForReadOnlies, shouldThrowExceptionForMutables,
            shouldChangeFrameOfMutables, m -> true);
   }

   public static void assertStaticMethodsCheckReferenceFrame(Class<?> typeHoldingStaticMethodsToTest, boolean shouldThrowExceptionForReadOnlies,
         boolean shouldThrowExceptionForMutables, boolean shouldChangeFrameOfMutables, Predicate<Method> methodFilter) throws Throwable
   {
      // We need at least 2 frame arguments to assert anything.
      List<Method> frameMethods = keepOnlyMethodsWithAtLeastNFrameArguments(typeHoldingStaticMethodsToTest.getMethods(), 2);
      // We keep only the public & static methods
      frameMethods = frameMethods.stream().filter(m -> Modifier.isStatic(m.getModifiers())).filter(m -> Modifier.isPublic(m.getModifiers()))
            .collect(Collectors.toList());
      // Apply the custom filter
      frameMethods = frameMethods.stream().filter(methodFilter).collect(Collectors.toList());

      ReferenceFrame frameA = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("frameA", ReferenceFrame.getWorldFrame(),
            new RigidBodyTransform());
      ReferenceFrame frameB = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("frameB", ReferenceFrame.getWorldFrame(),
            new RigidBodyTransform());

      // First check that the method is fine with all the arguments in the same frame.
      for (Method frameMethod : frameMethods)
      {
         Class<?>[] parameterTypes = frameMethod.getParameterTypes();
         Object[] parameters = new Object[parameterTypes.length];

         for (int i = 0; i < parameterTypes.length; i++)
         {
            Class<?> parameterType = parameterTypes[i];
            instantiateParameterType(frameA, parameters, i, parameterType);
         }

         invokeStaticMethod(frameMethod, parameters);
      }

      // Check that the method checks the reference frames.
      for (Method frameMethod : frameMethods)
      {
         Class<?>[] parameterTypes = frameMethod.getParameterTypes();

         int numberOfArgumentsToTest = 0;
         for (Class<?> parameterType : parameterTypes)
         {
            if (shouldThrowExceptionForReadOnlies && isFrameTypeReadOnly(parameterType))
               numberOfArgumentsToTest++;
            if (shouldThrowExceptionForMutables && isFrameTypeMutable(parameterType))
               numberOfArgumentsToTest++;
         }
         int numberOfCombinations = (int) Math.pow(2, numberOfArgumentsToTest);

         for (int i = 1; i < numberOfCombinations - 1; i++)
         {
            Object[] parameters = new Object[parameterTypes.length];
            int currentByte = 0;

            for (int j = 0; j < parameterTypes.length; j++)
            {
               Class<?> parameterType = parameterTypes[j];
               boolean mutateFrame = shouldThrowExceptionForReadOnlies && isFrameTypeReadOnly(parameterType);
               mutateFrame |= shouldThrowExceptionForMutables && isFrameTypeMutable(parameterType);

               if (!mutateFrame)
               {
                  instantiateParameterType(frameA, parameters, j, parameterType);
               }
               else
               {
                  ReferenceFrame frame = frameA;
                  int mask = (int) Math.pow(2, currentByte);
                  if ((i & mask) != 0)
                     frame = frameB;
                  instantiateParameterType(frame, parameters, j, parameterType);
                  currentByte++;
               }
            }

            try
            {
               invokeStaticMethod(frameMethod, parameters);
               String message = "Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName();
               message += "\nType being tested: " + typeHoldingStaticMethodsToTest.getSimpleName();
               message += "\nMethod: " + getMethodSimpleName(frameMethod);
               message += "\nArguments used: " + Arrays.toString(parameters);
               message += "\nArgument types: " + getArgumentTypeString(parameters);
               Assert.fail(message);
            }
            catch (ReferenceFrameMismatchException e)
            {
               // Good
            }
         }
      }

      // Check that the frame of each mutable is changed (optional)
      if (shouldChangeFrameOfMutables)
      {
         for (Method frameMethod : frameMethods)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               if (isFrameTypeMutable(parameterType))
                  instantiateParameterType(frameB, parameters, i, parameterType);
               else
                  instantiateParameterType(frameA, parameters, i, parameterType);
            }

            invokeStaticMethod(frameMethod, parameters);

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               if (isFrameTypeMutable(parameterType))
               {
                  ReferenceFrame newFrame = ((ReferenceFrameHolder) parameters[i]).getReferenceFrame();
                  if (newFrame != frameA)
                  {
                     String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not change the frame of the " + i + "th parameter.";
                     message += "\nType being tested: " + typeHoldingStaticMethodsToTest.getSimpleName();
                     message += "\nArguments used: " + Arrays.toString(parameters);
                     message += "\nArgument types: " + getArgumentTypeString(parameters);
                     Assert.fail(message);
                  }
               }
            }
         }
      }
   }

   private static boolean isFrameTypeReadOnly(Class<?> frameType)
   {
      return frameReadOnlyTypes.contains(frameType) && !frameMutableTypes.contains(frameType);
   }

   private static boolean isFrameTypeMutable(Class<?> frameType)
   {
      return frameMutableTypes.contains(frameType);
   }

   private static void invokeStaticMethod(Method frameMethod, Object[] parameters) throws Throwable
   {
      try
      {
         frameMethod.invoke(null, parameters);
      }
      catch (IllegalAccessException | IllegalArgumentException e)
      {
         System.err.println("Something went wrong when involing the static method: " + getMethodSimpleName(frameMethod));
         System.err.println("Objects used as parameters: " + getArgumentTypeString(parameters));
         e.printStackTrace();
         throw e;
      }
      catch (InvocationTargetException e)
      {
         throw e.getCause();
      }
   }

   private static String getArgumentTypeString(Object[] arguments)
   {
      String string = "";
      for (int i = 0; i < arguments.length; i++)
      {
         string += arguments[i].getClass().getSimpleName();
         if (i < arguments.length - 1)
            string += ", ";
      }
      return string;
   }

   private static void assertMethodOverloadedWithSpecificSignature(Class<?> typeWithOverloadingMethods, Class<?> typeWithOriginalMethod, Method originalMethod,
         Class<?>[] overloadingSignature, Class<?> typeToSearchIn) throws SecurityException
   {
      try
      {
         Method overloadingMethod = typeToSearchIn.getMethod(originalMethod.getName(), overloadingSignature);
         Class<?> originalReturnType = originalMethod.getReturnType();
         Class<?> overloadingReturnType = overloadingMethod.getReturnType();

         { // Assert the return type is proper
            if (originalReturnType == null && overloadingReturnType != null)
            {
               String message = "Inconsistency found in the return type.";
               message += "\nOriginal method: " + getMethodSimpleName(originalMethod);
               message += "\nOverloading method: " + getMethodSimpleName(overloadingMethod);
               message += "\nOriginal type declaring method: " + typeWithOriginalMethod.getSimpleName();
               message += "\nType overloading original: " + typeWithOverloadingMethods.getSimpleName();
               throw new AssertionError(message);
            }

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

   private static List<Method> keepOnlyMethodsWithAtLeastNFrameArguments(Method[] methodsToFilter, int minNumberOfFrameArguments)
   {
      return keepOnlyMethodsWithAtLeastNFrameArguments(Arrays.asList(methodsToFilter), minNumberOfFrameArguments);
   }

   private static List<Method> keepOnlyMethodsWithAtLeastNFrameArguments(List<Method> methodsToFilter, int minNumberOfFrameArguments)
   {
      return methodsToFilter.stream().filter(m -> methodHasAtLeastNFrameArguments(m, minNumberOfFrameArguments)).collect(Collectors.toList());
   }

   private static boolean methodHasAtLeastNFrameArguments(Method method, int minNumberOfFrameArguments)
   {
      int numberOfFrameArguments = 0;

      for (Class<?> parameterType : method.getParameterTypes())
      {
         if (isFrameType(parameterType))
         {
            numberOfFrameArguments++;
            if (numberOfFrameArguments >= minNumberOfFrameArguments)
               return true;
         }
      }

      return numberOfFrameArguments >= minNumberOfFrameArguments;
   }

   private static List<Method> keepOnlyMethodsWithAtLeastOneGeometryArgumentAndNoFrames(Method[] methodsToFilter, int minNumberOfFramelessArguments)
   {
      return keepOnlyMethodsWithAtLeastNFramelessArgument(Arrays.asList(methodsToFilter), minNumberOfFramelessArguments);
   }

   private static List<Method> keepOnlyMethodsWithAtLeastNFramelessArgument(List<Method> methodsToFilter, int minNumberOfFramelessArguments)
   {
      return methodsToFilter.stream().filter(m -> methodHasAtLeastNFramelessArguments(m, minNumberOfFramelessArguments)).collect(Collectors.toList());
   }

   private static boolean methodHasAtLeastNFramelessArguments(Method method, int minNumberOfFramelessArguments)
   {
      int numberOfFramelessArguments = 0;

      for (Class<?> parameterType : method.getParameterTypes())
      {
         if (isFramelessType(parameterType))
         {
            numberOfFramelessArguments++;
            if (numberOfFramelessArguments >= minNumberOfFramelessArguments)
               return true;
         }
      }

      return numberOfFramelessArguments >= minNumberOfFramelessArguments;
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

   static void instantiateParameterType(ReferenceFrame frameA, Object[] parameters, int i, Class<?> parameterType)
   {
      if (isFrameType(parameterType))
         parameters[i] = createFrameObject(parameterType, frameA);
      else
         parameters[i] = newInstanceOf(parameterType);
   }

   private static Object createFrameObject(Class<?> type, ReferenceFrame referenceFrame)
   {
      FrameTypeBuilder builder = null;
      Class<?> bestMatchingType = null;

      for (Entry<Class<?>, FrameTypeBuilder> entry : frameTypeBuilders.entrySet())
      {
         if (!entry.getKey().isAssignableFrom(type))
            continue;

         if (bestMatchingType == null || bestMatchingType.isAssignableFrom(entry.getKey()))
         {
            bestMatchingType = entry.getKey();
            builder = entry.getValue();
         }
      }

      return builder.newInstance(referenceFrame);
   }

   private static interface FrameTypeBuilder
   {
      Object newInstance(ReferenceFrame referenceFrame);
   }

   private static interface GenericTypeBuilder
   {
      Object newInstance();
   }

   private static Object newInstanceOf(Class<?> type)
   {
      if (genericTypeBuilders.containsKey(type))
         return genericTypeBuilders.get(type).newInstance();

      if (type.isPrimitive())
      {
         if (type.equals(Boolean.TYPE))
            return false;
         else
            return 0;
      }

      try
      {
         return type.getConstructor().newInstance();
      }
      catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException | NoSuchMethodException
            | SecurityException e)
      {
         throw new RuntimeException("Could instantiate an object of the type: " + type.getSimpleName());
      }
   }
}
