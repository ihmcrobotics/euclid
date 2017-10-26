package us.ihmc.euclid.referenceFrame.tools;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.*;
import us.ihmc.euclid.tuple3D.interfaces.*;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.*;
import java.util.Map.Entry;
import java.util.function.Predicate;
import java.util.stream.Collectors;

public class EuclidFrameAPITestTools
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean DEBUG = false;
   private final static int FRAME_CHECK_ITERATIONS = 100;
   private final static int FUNCTIONALITY_ITERATIONS = 500;
   private final static Random random = new Random(345345);
   private final static double epsilon = 1.0e-12;

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

      modifiableMap.put(QuaternionReadOnly.class, FrameQuaternionReadOnly.class);
      modifiableMap.put(QuaternionBasics.class, FrameQuaternion.class);

      framelessTypesToFrameTypesTable = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Map<Class<?>, RandomFrameTypeBuilder<?>> frameTypeBuilders;
   static
   {
      HashMap<Class<?>, RandomFrameTypeBuilder<?>> modifiableMap = new HashMap<>();
      modifiableMap.put(FrameTuple2DReadOnly.class, frame -> EuclidFrameRandomTools.generateRandomFramePoint2D(random, frame));
      modifiableMap.put(FrameTuple2D.class, frame -> EuclidFrameRandomTools.generateRandomFramePoint2D(random, frame));
      modifiableMap.put(FramePoint2DReadOnly.class, frame -> EuclidFrameRandomTools.generateRandomFramePoint2D(random, frame));
      modifiableMap.put(FramePoint2D.class, frame -> EuclidFrameRandomTools.generateRandomFramePoint2D(random, frame));
      modifiableMap.put(FrameVector2DReadOnly.class, frame -> EuclidFrameRandomTools.generateRandomFrameVector2D(random, frame));
      modifiableMap.put(FrameVector2D.class, frame -> EuclidFrameRandomTools.generateRandomFrameVector2D(random, frame));

      modifiableMap.put(FrameTuple3DReadOnly.class, frame -> EuclidFrameRandomTools.generateRandomFramePoint3D(random, frame));
      modifiableMap.put(FrameTuple3D.class, frame -> EuclidFrameRandomTools.generateRandomFramePoint3D(random, frame));
      modifiableMap.put(FramePoint3DReadOnly.class, frame -> EuclidFrameRandomTools.generateRandomFramePoint3D(random, frame));
      modifiableMap.put(FramePoint3D.class, frame -> EuclidFrameRandomTools.generateRandomFramePoint3D(random, frame));
      modifiableMap.put(FrameVector3DReadOnly.class, frame -> EuclidFrameRandomTools.generateRandomFrameVector3D(random, frame));
      modifiableMap.put(FrameVector3D.class, frame -> EuclidFrameRandomTools.generateRandomFrameVector3D(random, frame));

      modifiableMap.put(FrameQuaternionReadOnly.class, frame -> EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame));
      modifiableMap.put(FrameQuaternion.class, frame -> EuclidFrameRandomTools.generateRandomFrameQuaternion(random, frame));

      frameTypeBuilders = Collections.unmodifiableMap(modifiableMap);
   }

   private final static Map<Class<?>, GenericTypeBuilder> framelessTypeBuilders;
   static
   {
      HashMap<Class<?>, GenericTypeBuilder> modifiableMap = new HashMap<>();
      modifiableMap.put(Tuple2DReadOnly.class, () -> EuclidCoreRandomTools.generateRandomPoint2D(random));
      modifiableMap.put(Tuple2DBasics.class, () -> EuclidCoreRandomTools.generateRandomPoint2D(random));
      modifiableMap.put(Point2DReadOnly.class, () -> EuclidCoreRandomTools.generateRandomPoint2D(random));
      modifiableMap.put(Point2DBasics.class, () -> EuclidCoreRandomTools.generateRandomPoint2D(random));
      modifiableMap.put(Vector2DReadOnly.class, () -> EuclidCoreRandomTools.generateRandomVector2D(random));
      modifiableMap.put(Vector2DBasics.class, () -> EuclidCoreRandomTools.generateRandomVector2D(random));

      modifiableMap.put(Tuple3DReadOnly.class, () -> EuclidCoreRandomTools.generateRandomPoint3D(random));
      modifiableMap.put(Tuple3DBasics.class, () -> EuclidCoreRandomTools.generateRandomPoint3D(random));
      modifiableMap.put(Point3DReadOnly.class, () -> EuclidCoreRandomTools.generateRandomPoint3D(random));
      modifiableMap.put(Point3DBasics.class, () -> EuclidCoreRandomTools.generateRandomPoint3D(random));
      modifiableMap.put(Vector3DReadOnly.class, () -> EuclidCoreRandomTools.generateRandomVector3D(random));
      modifiableMap.put(Vector3DBasics.class, () -> EuclidCoreRandomTools.generateRandomVector3D(random));

      modifiableMap.put(AxisAngleReadOnly.class, () -> EuclidCoreRandomTools.generateRandomAxisAngle(random));

      modifiableMap.put(Tuple4DReadOnly.class, () -> EuclidCoreRandomTools.generateRandomQuaternion(random));
      modifiableMap.put(Vector4DReadOnly.class, () -> EuclidCoreRandomTools.generateRandomVector4D(random));
      modifiableMap.put(RotationMatrixReadOnly.class, () -> EuclidCoreRandomTools.generateRandomRotationMatrix(random));
      modifiableMap.put(Matrix3DReadOnly.class, () -> EuclidCoreRandomTools.generateRandomMatrix3D(random));
      modifiableMap.put(QuaternionReadOnly.class, () -> EuclidCoreRandomTools.generateRandomQuaternion(random));
      modifiableMap.put(QuaternionBasics.class, () -> EuclidCoreRandomTools.generateRandomQuaternion(random));

      framelessTypeBuilders = Collections.unmodifiableMap(modifiableMap);
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
      modifiableSet.add(FrameQuaternionReadOnly.class);

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
      modifiableSet.add(FrameQuaternion.class);

      frameMutableTypes = Collections.unmodifiableSet(modifiableSet);
   }

   private final static Set<Class<?>> acceptableExceptions;
   static
   {
      Set<Class<?>> modifiableSet = new HashSet<>();
      modifiableSet.add(BoundingBoxException.class);
      modifiableSet.add(IllegalArgumentException.class);
      modifiableSet.add(RuntimeException.class);

      acceptableExceptions = Collections.unmodifiableSet(modifiableSet);
   }

   /**
    * Asserts, using reflection, that all methods with frameless arguments, such as
    * {@code Tuple3DReadOnly}, are overloaded with their frame type equivalent, i.e.
    * {@code Tuple2DBasics} is to be overloaded with {@code FrameTuple2D}.
    * 
    * @param typeWithFrameMethods refers to the type to be tested. This asserts that
    *           {@code typeWithFrameMethods} properly has all the methods necessary to properly
    *           overload {@code typeWithFramelessMethods}.
    * @param typeWithFramelessMethods refers to the type declaring methods with frameless objects
    *           that are to be overloaded.
    * @param assertAllCombinations when {@code false}, this asserts that for each method in
    *           {@code typeWithFramelessMethods} there is one overloading method in
    *           {@code typeWithFrameMethods} with all the arguments using the equivalent frame type.
    *           When {@code true}, this asserts that for each method in
    *           {@code typeWithFramelessArguments}, {@code typeWithFrameMethods} overloads it with
    *           all the possible combinations of frame & frameless arguments, except for the
    *           original frameless signature.
    */
   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods, boolean assertAllCombinations)
   {
      assertOverloadingWithFrameObjects(typeWithFrameMethods, typeWithFramelessMethods, assertAllCombinations, 1);
   }

   /**
    * Asserts, using reflection, that all methods with frameless arguments, such as
    * {@code Tuple3DReadOnly}, are overloaded with their frame type equivalent, i.e.
    * {@code Tuple2DBasics} is to be overloaded with {@code FrameTuple2D}.
    * 
    * @param typeWithFrameMethods refers to the type to be tested. This asserts that
    *           {@code typeWithFrameMethods} properly has all the methods necessary to properly
    *           overload {@code typeWithFramelessMethods}.
    * @param typeWithFramelessMethods refers to the type declaring methods with frameless objects
    *           that are to be overloaded.
    * @param assertAllCombinations when {@code false}, this asserts that for each method in
    *           {@code typeWithFramelessMethods} there is one overloading method in
    *           {@code typeWithFrameMethods} with all the arguments using the equivalent frame type.
    *           When {@code true}, this asserts that for each method in
    *           {@code typeWithFramelessArguments}, {@code typeWithFrameMethods} overloads it with
    *           all the possible combinations of frame & frameless arguments, except for the
    *           original frameless signature.
    * @param minNumberOfFramelessArguments threshold used to filter out methods to assert in
    *           {@code typeWithFramelessMethods}.
    */
   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods, boolean assertAllCombinations,
                                                        int minNumberOfFramelessArguments)
   {
      assertOverloadingWithFrameObjects(typeWithFrameMethods, typeWithFramelessMethods, assertAllCombinations, minNumberOfFramelessArguments, m -> true);
   }

   /**
    * Asserts, using reflection, that all methods with frameless arguments, such as
    * {@code Tuple3DReadOnly}, are overloaded with their frame type equivalent, i.e.
    * {@code Tuple2DBasics} is to be overloaded with {@code FrameTuple2D}.
    * 
    * @param typeWithFrameMethods refers to the type to be tested. This asserts that
    *           {@code typeWithFrameMethods} properly has all the methods necessary to properly
    *           overload {@code typeWithFramelessMethods}.
    * @param typeWithFramelessMethods refers to the type declaring methods with frameless objects
    *           that are to be overloaded.
    * @param assertAllCombinations when {@code false}, this asserts that for each method in
    *           {@code typeWithFramelessMethods} there is one overloading method in
    *           {@code typeWithFrameMethods} with all the arguments using the equivalent frame type.
    *           When {@code true}, this asserts that for each method in
    *           {@code typeWithFramelessArguments}, {@code typeWithFrameMethods} overloads it with
    *           all the possible combinations of frame & frameless arguments, except for the
    *           original frameless signature.
    * @param minNumberOfFramelessArguments threshold used to filter out methods to assert in
    *           {@code typeWithFramelessMethods}.
    * @param framelessMethodsToIgnore map containing the name and argument types of the methods in
    *           {@code typeWithFramelessMethods} to be ignored in this test.
    */
   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods, boolean assertAllCombinations,
                                                        int minNumberOfFramelessArguments, Map<String, Class<?>[]> framelessMethodsToIgnore)
   {
      Predicate<Method> methodFilter = new Predicate<Method>()
      {
         @Override
         public boolean test(Method m)
         {
            for (Entry<String, Class<?>[]> methodToIgnore : framelessMethodsToIgnore.entrySet())
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

      assertOverloadingWithFrameObjects(typeWithFrameMethods, typeWithFramelessMethods, assertAllCombinations, minNumberOfFramelessArguments, methodFilter);
   }

   /**
    * Asserts, using reflection, that all methods with frameless arguments, such as
    * {@code Tuple3DReadOnly}, are overloaded with their frame type equivalent, i.e.
    * {@code Tuple2DBasics} is to be overloaded with {@code FrameTuple2D}.
    * 
    * @param typeWithFrameMethods refers to the type to be tested. This asserts that
    *           {@code typeWithFrameMethods} properly has all the methods necessary to properly
    *           overload {@code typeWithFramelessMethods}.
    * @param typeWithFramelessMethods refers to the type declaring methods with frameless objects
    *           that are to be overloaded.
    * @param assertAllCombinations when {@code false}, this asserts that for each method in
    *           {@code typeWithFramelessMethods} there is one overloading method in
    *           {@code typeWithFrameMethods} with all the arguments using the equivalent frame type.
    *           When {@code true}, this asserts that for each method in
    *           {@code typeWithFramelessArguments}, {@code typeWithFrameMethods} overloads it with
    *           all the possible combinations of frame & frameless arguments, except for the
    *           original frameless signature.
    * @param minNumberOfFramelessArguments threshold used to filter out methods to assert in
    *           {@code typeWithFramelessMethods}.
    * @param framelessMethodFilter custom filter used on the methods of
    *           {@code typeWithFramelessMethods}. The assertions are performed on the methods for
    *           which {@code framelessMethodFilter.test(method)} returns {@code true}.
    */
   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods, boolean assertAllCombinations,
                                                        int minNumberOfFramelessArguments, Predicate<Method> framelessMethodFilter)
   {
      // The frame methods are all the methods from 'typeWithFramelessMethods' that have at least one geometry argument.
      List<Method> framelessMethods = keepOnlyMethodsWithAtLeastNFramelessArguments(typeWithFramelessMethods.getMethods(), minNumberOfFramelessArguments);

      for (Method framelessMethod : framelessMethods)
      {
         if (framelessMethodFilter.test(framelessMethod))
         {
            // Creating all the expected combinations
            List<Class<?>[]> expectedMethodSignatures = createExpectedMethodSignaturesWithFrameArgument(framelessMethod, assertAllCombinations);

            for (Class<?>[] expectedMethodSignature : expectedMethodSignatures)
            {
               assertMethodOverloadedWithSpecificSignature(typeWithFrameMethods, typeWithFramelessMethods, framelessMethod, expectedMethodSignature,
                                                           typeWithFrameMethods);
            }
         }
      }
   }

   /**
    * Asserts, using reflection, that the methods, that are public and static, in
    * {@code typeHoldingStaticMethodsToTest} are properly checking and/or setting reference frames
    * of their arguments.
    * <p>
    * This assertion expects methods to be declaring arguments as read-only to inform that they are
    * used as input only, and as mutable to inform that they are the output(s).
    * </p>
    * <p>
    * Note that this does not perform any assertion for methods with only 1 frame argument.
    * </p>
    * <p>
    * This expects methods to throw a {@link ReferenceFrameMismatchException} to indicate that the
    * operation cannot be performed because at least two arguments are expressed in a different
    * reference frame.
    * </p>
    * 
    * @param typeDeclaringStaticMethodsToTest the type in which the methods are to be tested.
    * @param shouldThrowExceptionForMutables indicates that the methods should throw a
    *           {@link ReferenceFrameMismatchException} when at least two of the mutable arguments,
    *           for instance {@code FramePoint3D} or {@code FrameTuple2D}, are expressed in a
    *           different frame.
    * @param shouldChangeFrameOfMutables indicates that the methods should set the reference frame
    *           of the mutable arguments, such as {@code FramePoint3D} or {@code FrameTuple2D}. This
    *           option is incompatible with {@code shouldThrowExceptionForMutables}.
    * @throws IllegalArgumentException if {@code shouldChangeFrameOfMutables} and
    *            {@code shouldThrowExceptionForMutables} are both set to {@code true}.
    * @throws IllegalArgumentException if {@code shouldChangeFrameOfMutables} is set to {@code true}
    *            and {@code shouldThrowExceptionForReadOnlies} is set to {@code false}.
    * @throws Throwable if an unexpected throwable has been thrown by a method at invocation time.
    */
   public static void assertStaticMethodsCheckReferenceFrame(Class<?> typeDeclaringStaticMethodsToTest, boolean shouldThrowExceptionForMutables,
                                                             boolean shouldChangeFrameOfMutables)
         throws Throwable
   {
      assertStaticMethodsCheckReferenceFrame(typeDeclaringStaticMethodsToTest, shouldThrowExceptionForMutables, shouldChangeFrameOfMutables, m -> true);
   }

   /**
    * Asserts, using reflection, that the methods, that are public and static, in
    * {@code typeHoldingStaticMethodsToTest} are properly checking and/or setting reference frames
    * of their arguments.
    * <p>
    * This assertion expects methods to be declaring arguments as read-only to inform that they are
    * used as input only, and as mutable to inform that they are the output(s).
    * </p>
    * <p>
    * Note that this does not perform any assertion for methods with only 1 frame argument.
    * </p>
    * <p>
    * This expects methods to throw a {@link ReferenceFrameMismatchException} to indicate that the
    * operation cannot be performed because at least two arguments are expressed in a different
    * reference frame.
    * </p>
    * 
    * @param typeDeclaringStaticMethodsToTest the type in which the methods are to be tested.
    * @param shouldThrowExceptionForMutables indicates that the methods should throw a
    *           {@link ReferenceFrameMismatchException} when at least two of the mutable arguments,
    *           for instance {@code FramePoint3D} or {@code FrameTuple2D}, are expressed in a
    *           different frame.
    * @param shouldChangeFrameOfMutables indicates that the methods should set the reference frame
    *           of the mutable arguments, such as {@code FramePoint3D} or {@code FrameTuple2D}. This
    *           option is incompatible with {@code shouldThrowExceptionForMutables}.
    * @param methodFilter custom filter used on the methods. The assertions are performed on the
    *           methods for which {@code methodFilter.test(method)} returns {@code true}.
    * @throws IllegalArgumentException if {@code shouldChangeFrameOfMutables} and
    *            {@code shouldThrowExceptionForMutables} are both set to {@code true}.
    * @throws IllegalArgumentException if {@code shouldChangeFrameOfMutables} is set to {@code true}
    *            and {@code shouldThrowExceptionForReadOnlies} is set to {@code false}.
    * @throws Throwable if an unexpected throwable has been thrown by a method at invocation time.
    */
   public static void assertStaticMethodsCheckReferenceFrame(Class<?> typeDeclaringStaticMethodsToTest, boolean shouldThrowExceptionForMutables,
                                                             boolean shouldChangeFrameOfMutables, Predicate<Method> methodFilter)
         throws Throwable, IllegalArgumentException
   {
      if (shouldThrowExceptionForMutables && shouldChangeFrameOfMutables)
         throw new IllegalArgumentException("Incompatible selection. A method cannot check reference frames of mutable argument AND set their reference frame.");

      // We need at least 2 frame arguments to assert anything.
      List<Method> frameMethods = keepOnlyMethodsWithAtLeastNFrameArguments(typeDeclaringStaticMethodsToTest.getMethods(), 2);
      // We keep only the public & static methods
      frameMethods = frameMethods.stream().filter(m -> Modifier.isStatic(m.getModifiers())).filter(m -> Modifier.isPublic(m.getModifiers()))
                                 .collect(Collectors.toList());
      // Apply the custom filter
      frameMethods = frameMethods.stream().filter(methodFilter).collect(Collectors.toList());
      // Methods returning a frame type
      List<Method> methodsWithReturnFrameType = frameMethods.stream().filter(m -> isFrameType(m.getReturnType())).collect(Collectors.toList());

      for (int iteration = 0; iteration < FRAME_CHECK_ITERATIONS; iteration++)
      {
         ReferenceFrame frameA = EuclidFrameRandomTools.generateRandomReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.generateRandomReferenceFrame("frameB", random, worldFrame);

         // First check that the method is fine with all the arguments in the same frame.
         for (Method frameMethod : frameMethods)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               parameters[i] = instantiateParameterType(frameA, parameterType);
            }

            try
            {
               invokeStaticMethod(frameMethod, parameters);
            }
            catch (Throwable t)
            {
               if (!isExceptionAcceptable(t))
                  throw t;
            }
         }

         // Check that the method checks the reference frames.
         for (Method frameMethod : frameMethods)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();

            int numberOfArgumentsToTest = 0;
            for (Class<?> parameterType : parameterTypes)
            {
               if (isFrameTypeReadOnly(parameterType))
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
                  boolean mutateFrame = isFrameTypeReadOnly(parameterType);
                  mutateFrame |= shouldThrowExceptionForMutables && isFrameTypeMutable(parameterType);

                  if (!mutateFrame)
                  {
                     parameters[j] = instantiateParameterType(frameA, parameterType);
                  }
                  else
                  {
                     ReferenceFrame frame = frameA;
                     int mask = (int) Math.pow(2, currentByte);
                     if ((i & mask) != 0)
                        frame = frameB;
                     parameters[j] = instantiateParameterType(frame, parameterType);
                     currentByte++;
                  }
               }

               try
               {
                  invokeStaticMethod(frameMethod, parameters);
                  String message = "Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName();
                  message += "\nType being tested: " + typeDeclaringStaticMethodsToTest.getSimpleName();
                  message += "\nMethod: " + getMethodSimpleName(frameMethod);
                  message += "\nArguments used: " + Arrays.toString(parameters);
                  message += "\nArgument types: " + getArgumentTypeString(parameters);
                  throw new AssertionError(message);
               }
               catch (ReferenceFrameMismatchException e)
               {
                  // Good
               }
               catch (Throwable t)
               {
                  if (!isExceptionAcceptable(t))
                     throw t;
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
                     parameters[i] = instantiateParameterType(frameB, parameterType);
                  else
                     parameters[i] = instantiateParameterType(frameA, parameterType);
               }

               try
               {
                  invokeStaticMethod(frameMethod, parameters);
               }
               catch (Throwable t)
               {
                  if (!isExceptionAcceptable(t))
                     throw t;
                  else
                     continue;
               }

               for (int i = 0; i < parameterTypes.length; i++)
               {
                  Class<?> parameterType = parameterTypes[i];
                  if (isFrameTypeMutable(parameterType))
                  {
                     ReferenceFrame newFrame = ((ReferenceFrameHolder) parameters[i]).getReferenceFrame();
                     if (newFrame != frameA)
                     {
                        String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not change the frame of the " + (i + 1) + "th parameter.";
                        message += "\nType being tested: " + typeDeclaringStaticMethodsToTest.getSimpleName();
                        message += "\nArguments used: " + Arrays.toString(parameters);
                        message += "\nArgument types: " + getArgumentTypeString(parameters);
                        throw new AssertionError(message);
                     }
                  }
               }
            }
         }

         // Check for methods returning a frame type that the reference frame is properly set.
         for (Method frameMethod : methodsWithReturnFrameType)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               parameters[i] = instantiateParameterType(frameA, parameterType);
            }

            Object result = null;

            try
            {
               result = invokeStaticMethod(frameMethod, parameters);
            }
            catch (Throwable t)
            {
               if (!isExceptionAcceptable(t))
                  throw t;
            }

            if (result == null)
               continue;

            ReferenceFrame resultFrame = ((ReferenceFrameHolder) result).getReferenceFrame();
            if (resultFrame != frameA)
            {
               String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not set the frame of the result.";
               message += "\nType being tested: " + typeDeclaringStaticMethodsToTest.getSimpleName();
               message += "\nArguments used: " + Arrays.toString(parameters);
               message += "\nArgument types: " + getArgumentTypeString(parameters);
               message += "\nResult: " + result;
               throw new AssertionError(message);
            }
         }
      }
   }

   public static void assertMethodsOfReferenceFrameHolderCheckReferenceFrame(RandomFrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder,
                                                                             boolean shouldThrowExceptionForMutables, boolean shouldChangeFrameOfMutables,
                                                                             Predicate<Method> methodFilter)
         throws Throwable, IllegalArgumentException
   {
      if (shouldThrowExceptionForMutables && shouldChangeFrameOfMutables)
         throw new IllegalArgumentException("Incompatible selection. A method cannot check reference frames of mutable argument AND set their reference frame.");

      Class<? extends ReferenceFrameHolder> frameType = frameTypeBuilder.newInstance(worldFrame).getClass();

      // We need at least 1 frame arguments to assert anything.
      List<Method> frameMethods = keepOnlyMethodsWithAtLeastNFrameArguments(frameType.getMethods(), 1);
      // We keep only the public & static methods
      frameMethods = frameMethods.stream().filter(m -> Modifier.isPublic(m.getModifiers())).collect(Collectors.toList());
      // Apply the custom filter
      frameMethods = frameMethods.stream().filter(methodFilter).collect(Collectors.toList());
      // Methods returning a frame type
      List<Method> methodsWithReturnFrameType = frameMethods.stream().filter(m -> isFrameType(m.getReturnType())).collect(Collectors.toList());

      for (int iteration = 0; iteration < FRAME_CHECK_ITERATIONS; iteration++)
      {
         ReferenceFrame frameA = EuclidFrameRandomTools.generateRandomReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.generateRandomReferenceFrame("frameB", random, worldFrame);
         ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(frameA);

         // First check that the method is fine with the holder and all the arguments in the same frame.
         for (Method frameMethod : frameMethods)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               parameters[i] = instantiateParameterType(frameA, parameterType);
            }

            try
            {
               invokeMethod(frameObject, frameMethod, parameters);
            }
            catch (Throwable t)
            {
               if (!isExceptionAcceptable(t))
                  throw t;
            }
         }

         // Check that the method checks the reference frames.
         for (Method frameMethod : frameMethods)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();

            int numberOfArgumentsToTest = 0;
            for (Class<?> parameterType : parameterTypes)
            {
               if (isFrameTypeReadOnly(parameterType))
                  numberOfArgumentsToTest++;
               if (shouldThrowExceptionForMutables && isFrameTypeMutable(parameterType))
                  numberOfArgumentsToTest++;
            }
            int numberOfCombinations = (int) Math.pow(2, numberOfArgumentsToTest);

            for (int i = 1; i < numberOfCombinations; i++)
            {
               Object[] parameters = new Object[parameterTypes.length];
               int currentByte = 0;

               for (int j = 0; j < parameterTypes.length; j++)
               {
                  Class<?> parameterType = parameterTypes[j];
                  boolean mutateFrame = isFrameTypeReadOnly(parameterType);
                  mutateFrame |= shouldThrowExceptionForMutables && isFrameTypeMutable(parameterType);

                  if (!mutateFrame)
                  {
                     parameters[j] = instantiateParameterType(frameA, parameterType);
                  }
                  else
                  {
                     ReferenceFrame frame = frameA;
                     int mask = (int) Math.pow(2, currentByte);
                     if ((i & mask) != 0)
                        frame = frameB;
                     parameters[j] = instantiateParameterType(frame, parameterType);
                     currentByte++;
                  }
               }

               try
               {
                  invokeMethod(frameObject, frameMethod, parameters);
                  String message = "Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName();
                  message += "\nType being tested: " + frameType.getSimpleName();
                  message += "\nMethod: " + getMethodSimpleName(frameMethod);
                  message += "\nArguments used: " + Arrays.toString(parameters);
                  message += "\nArgument types: " + getArgumentTypeString(parameters);
                  throw new AssertionError(message);
               }
               catch (ReferenceFrameMismatchException e)
               {
                  // Good
               }
               catch (Throwable t)
               {
                  if (!isExceptionAcceptable(t))
                     throw t;
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
                     parameters[i] = instantiateParameterType(frameB, parameterType);
                  else
                     parameters[i] = instantiateParameterType(frameA, parameterType);
               }

               try
               {
                  invokeMethod(frameObject, frameMethod, parameters);
               }
               catch (Throwable t)
               {
                  if (!isExceptionAcceptable(t))
                     throw t;
                  else
                     continue;
               }

               for (int i = 0; i < parameterTypes.length; i++)
               {
                  Class<?> parameterType = parameterTypes[i];
                  if (isFrameTypeMutable(parameterType))
                  {
                     ReferenceFrame newFrame = ((ReferenceFrameHolder) parameters[i]).getReferenceFrame();
                     if (newFrame != frameA)
                     {
                        String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not change the frame of the " + (i + 1) + "th parameter.";
                        message += "\nType being tested: " + frameType.getSimpleName();
                        message += "\nArguments used: " + Arrays.toString(parameters);
                        message += "\nArgument types: " + getArgumentTypeString(parameters);
                        throw new AssertionError(message);
                     }
                  }
               }
            }
         }

         // Check for methods returning a frame type that the reference frame is properly set.
         for (Method frameMethod : methodsWithReturnFrameType)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               parameters[i] = instantiateParameterType(frameA, parameterType);
            }

            Object result = null;

            try
            {
               result = invokeMethod(frameObject, frameMethod, parameters);
            }
            catch (Throwable t)
            {
               if (!isExceptionAcceptable(t))
                  throw t;
            }

            if (result == null)
               continue;

            ReferenceFrame resultFrame = ((ReferenceFrameHolder) result).getReferenceFrame();
            if (resultFrame != frameA)
            {
               String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not set the frame of the result.";
               message += "\nType being tested: " + frameType.getSimpleName();
               message += "\nArguments used: " + Arrays.toString(parameters);
               message += "\nArgument types: " + getArgumentTypeString(parameters);
               message += "\nResult: " + result;
               throw new AssertionError(message);
            }
         }
      }
   }

   /**
    * Assuming the type {@code typeWithFrameMethodsToTest} declares the same static methods as
    * declared in {@code typeWithFramlessMethods} with the difference of dealing with reference
    * frame holders, this method asserts that the methods in {@code typeWithFrameMethodsToTest} does
    * not change the underlying algorithms.
    * <p>
    * For each method declared in {@code typeWithFrameMethodsToTest}, this methods searched for the
    * equivalent method in {@code typeWithFramelessMethods} and the methods from both classes are
    * invoked to compare the output.
    * </p>
    * 
    * @param typeWithFrameMethodsToTest the type in which the methods are to be tested.
    * @param typeWithFramelessMethods the type declaring the methods against which the methods from
    *           {@code typeWithFrameMethodsToTest} are to be compared.
    */
   public static void assertStaticMethodsPreserveFunctionality(Class<?> typeWithFrameMethodsToTest, Class<?> typeWithFramelessMethods)
   {
      assertStaticMethodsPreserveFunctionality(typeWithFrameMethodsToTest, typeWithFramelessMethods, m -> true);
   }

   /**
    * Assuming the type {@code typeWithFrameMethodsToTest} declares the same static methods as
    * declared in {@code typeWithFramlessMethods} with the difference of dealing with reference
    * frame holders, this method asserts that the methods in {@code typeWithFrameMethodsToTest} does
    * not change the underlying algorithms.
    * <p>
    * For each method declared in {@code typeWithFrameMethodsToTest}, this methods searched for the
    * equivalent method in {@code typeWithFramelessMethods} and the methods from both classes are
    * invoked to compare the output.
    * </p>
    * 
    * @param typeWithFrameMethodsToTest the type in which the methods are to be tested.
    * @param typeWithFramelessMethods the type declaring the methods against which the methods from
    *           {@code typeWithFrameMethodsToTest} are to be compared.
    * @param methodFilter custom filter used on the methods. The assertions are performed on the
    *           methods for which {@code methodFilter.test(method)} returns {@code true}.
    */
   public static void assertStaticMethodsPreserveFunctionality(Class<?> typeWithFrameMethodsToTest, Class<?> typeWithFramelessMethods,
                                                               Predicate<Method> methodFilter)
   {
      List<Method> frameMethods = keepOnlyMethodsWithAtLeastNFrameArguments(typeWithFrameMethodsToTest.getMethods(), 0);

      for (Method frameMethod : frameMethods)
      {
         if (!methodFilter.test(frameMethod))
            continue;

         String frameMethodName = frameMethod.getName();
         Class<?>[] frameMethodParameterTypes = frameMethod.getParameterTypes();
         Class<?>[] framelessMethodParameterTypes = new Class[frameMethodParameterTypes.length];

         for (int i = 0; i < framelessMethodParameterTypes.length; i++)
         {
            if (isFrameType(frameMethodParameterTypes[i]))
               framelessMethodParameterTypes[i] = findCorrespondingFramelessType(frameMethodParameterTypes[i]);
            else
               framelessMethodParameterTypes[i] = frameMethodParameterTypes[i];
         }

         for (int iteration = 0; iteration < FUNCTIONALITY_ITERATIONS; iteration++)
         {
            try
            {
               Method framelessMethod = typeWithFramelessMethods.getMethod(frameMethodName, framelessMethodParameterTypes);
               Object[] frameMethodParameters = instantiateParameterTypes(worldFrame, frameMethodParameterTypes);

               if (frameMethodParameters == null)
               {
                  if (DEBUG)
                  {
                     String message = "Could not instantiate the parameters for the method: " + getMethodSimpleName(frameMethod)
                           + ". The method is not tested.";
                     System.err.println(message);
                  }
                  break;
               }

               Object[] framelessMethodParameters = clone(frameMethodParameters);
               Throwable expectedException = null;
               Object framelessMethodReturnObject = null;
               Object frameMethodReturnObject = null;

               try
               {
                  framelessMethodReturnObject = invokeStaticMethod(framelessMethod, framelessMethodParameters);
               }
               catch (Throwable e)
               {
                  expectedException = e;
               }

               try
               {
                  frameMethodReturnObject = invokeStaticMethod(frameMethod, frameMethodParameters);
               }
               catch (Throwable e)
               {
                  if (e.getClass() != expectedException.getClass())
                  {
                     String message = "";
                     message += "The method: " + getMethodSimpleName(frameMethod);
                     message += "\ndid not throw the same exception as the original method: " + getMethodSimpleName(framelessMethod);
                     message += "\nExpected exception class: " + expectedException.getClass().getSimpleName();
                     message += "\nActual exception class: " + e.getClass().getSimpleName();
                     throw new AssertionError(message);
                  }
                  else
                  {
                     continue;
                  }
               }

               for (int i = 0; i < frameMethodParameters.length; i++)
               {
                  Object framelessParameter = framelessMethodParameters[i];
                  Object frameParameter = frameMethodParameters[i];

                  if (!epsilonEquals(framelessParameter, frameParameter, epsilon))
                  {
                     String message = "";
                     message += "Detected a frame method inconsistent with its original frameless method.";
                     message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
                     message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
                     message += "\nFrame arguments after call:\n" + Arrays.toString(frameMethodParameters);
                     message += "\nFrameless arguments after call:\n" + toStringAsFramelessObjects(framelessMethodParameters);
                     throw new AssertionError(message);
                  }
               }

               if (!epsilonEquals(framelessMethodReturnObject, frameMethodReturnObject, epsilon))
               {
                  String message = "";
                  message += "Detected a frame method inconsistent with its original frameless method.";
                  message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
                  message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
                  message += "\nFrame method returned:" + frameMethodReturnObject;
                  message += "\nFrameless method returned:" + toStringAsFramelessObject(framelessMethodReturnObject);
                  throw new AssertionError(message);
               }
            }
            catch (NoSuchMethodException e)
            {
               if (DEBUG)
               {
                  String message = "";
                  message += "-------------------------------------------------------------------";
                  message += "\nCould not find the corresponding method: " + getMethodSimpleName(frameMethod);
                  message += "\nMethod is from type: " + typeWithFrameMethodsToTest.getSimpleName();
                  message += "\nSearched in: " + typeWithFramelessMethods.getSimpleName();
                  message += "\nSearched with argument type: " + getSimpleNames(framelessMethodParameterTypes);
                  message += "\n-------------------------------------------------------------------";
                  System.err.println(message);
               }
            }
            catch (SecurityException e)
            {
               if (DEBUG)
               {
                  String message = "";
                  message += "-------------------------------------------------------------------";
                  message += "\nUnable to access method with name: " + frameMethodName + " and argument types: "
                        + getSimpleNames(framelessMethodParameterTypes);
                  message += "\nin type: " + typeWithFramelessMethods.getSimpleName();
                  message += "\n-------------------------------------------------------------------";
                  System.err.println(message);
               }
            }
         }
      }
   }

   public static void assertFrameMethodsOfFrameHolderPreserveFunctionality(FrameTypeBuilder<? extends ReferenceFrameHolder> frameTypeBuilder,
                                                                           GenericTypeBuilder framelessTypeBuilber, Predicate<Method> methodFilter)
   {

      Class<? extends ReferenceFrameHolder> frameTypeToTest = frameTypeBuilder.newInstance(worldFrame, framelessTypeBuilber.newInstance()).getClass();
      Class<? extends Object> framelessType = framelessTypeBuilber.newInstance().getClass();

      List<Method> frameMethods = keepOnlyMethodsWithAtLeastNFrameArguments(frameTypeToTest.getMethods(), 0);

      for (Method frameMethod : frameMethods)
      {
         if (!methodFilter.test(frameMethod))
            continue;

         String frameMethodName = frameMethod.getName();
         Class<?>[] frameMethodParameterTypes = frameMethod.getParameterTypes();
         Class<?>[] framelessMethodParameterTypes = new Class[frameMethodParameterTypes.length];

         for (int i = 0; i < framelessMethodParameterTypes.length; i++)
         {
            if (isFrameType(frameMethodParameterTypes[i]))
               framelessMethodParameterTypes[i] = findCorrespondingFramelessType(frameMethodParameterTypes[i]);
            else
               framelessMethodParameterTypes[i] = frameMethodParameterTypes[i];
         }

         for (int iteration = 0; iteration < FUNCTIONALITY_ITERATIONS; iteration++)
         {
            Object framelessObject = framelessTypeBuilber.newInstance();
            ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(worldFrame, framelessObject);

            try
            {
               Method framelessMethod = framelessType.getMethod(frameMethodName, framelessMethodParameterTypes);
               Object[] frameMethodParameters = instantiateParameterTypes(worldFrame, frameMethodParameterTypes);

               if (frameMethodParameters == null)
               {
                  if (DEBUG)
                  {
                     String message = "Could not instantiate the parameters for the method: " + getMethodSimpleName(frameMethod)
                           + ". The method is not tested.";
                     System.err.println(message);
                  }
                  break;
               }

               Object[] framelessMethodParameters = clone(frameMethodParameters);
               Throwable expectedException = null;
               Object framelessMethodReturnObject = null;
               Object frameMethodReturnObject = null;

               try
               {
                  framelessMethodReturnObject = invokeMethod(framelessObject, framelessMethod, framelessMethodParameters);
               }
               catch (Throwable e)
               {
                  expectedException = e;
               }

               try
               {
                  frameMethodReturnObject = invokeMethod(frameObject, frameMethod, frameMethodParameters);
               }
               catch (Throwable e)
               {
                  if (e.getClass() != expectedException.getClass())
                  {
                     String message = "";
                     message += "The method: " + getMethodSimpleName(frameMethod);
                     message += "\ndid not throw the same exception as the original method: " + getMethodSimpleName(framelessMethod);
                     message += "\nExpected exception class: " + expectedException.getClass().getSimpleName();
                     message += "\nActual exception class: " + e.getClass().getSimpleName();
                     throw new AssertionError(message);
                  }
                  else
                  {
                     continue;
                  }
               }

               for (int i = 0; i < frameMethodParameters.length; i++)
               {
                  Object framelessParameter = framelessMethodParameters[i];
                  Object frameParameter = frameMethodParameters[i];

                  if (!epsilonEquals(framelessParameter, frameParameter, epsilon))
                  {
                     String message = "";
                     message += "Detected a frame method inconsistent with its original frameless method.";
                     message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
                     message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
                     message += "\nFrame arguments after call:\n" + Arrays.toString(frameMethodParameters);
                     message += "\nFrameless arguments after call:\n" + toStringAsFramelessObjects(framelessMethodParameters);
                     throw new AssertionError(message);
                  }
               }

               if (!epsilonEquals(framelessMethodReturnObject, frameMethodReturnObject, epsilon))
               {
                  String message = "";
                  message += "Detected a frame method inconsistent with its original frameless method.";
                  message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
                  message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
                  message += "\nFrame method returned:" + frameMethodReturnObject;
                  message += "\nFrameless method returned:" + toStringAsFramelessObject(framelessMethodReturnObject);
                  throw new AssertionError(message);
               }

               if (!epsilonEquals(framelessObject, frameObject, epsilon))
               {
                  String message = "";
                  message += "Detected a frame method inconsistent with its original frameless method.";
                  message += "\nInconsistent frame method: " + getMethodSimpleName(frameMethod);
                  message += "\nOriginal frameless method: " + getMethodSimpleName(framelessMethod);
                  message += "\nFrame object after method call:" + frameObject;
                  message += "\nFrameless object after method call:" + framelessObject;
                  throw new AssertionError(message);
               }

            }
            catch (NoSuchMethodException e)
            {
               if (DEBUG)
               {
                  String message = "";
                  message += "-------------------------------------------------------------------";
                  message += "\nCould not find the corresponding method: " + getMethodSimpleName(frameMethod);
                  message += "\nMethod is from type: " + frameTypeToTest.getSimpleName();
                  message += "\nSearched in: " + framelessType.getSimpleName();
                  message += "\nSearched with argument type: " + getSimpleNames(framelessMethodParameterTypes);
                  message += "\n-------------------------------------------------------------------";
                  System.err.println(message);
               }
            }
            catch (SecurityException e)
            {
               if (DEBUG)
               {
                  String message = "";
                  message += "-------------------------------------------------------------------";
                  message += "\nUnable to access method with name: " + frameMethodName + " and argument types: "
                        + getSimpleNames(framelessMethodParameterTypes);
                  message += "\nin type: " + framelessType.getSimpleName();
                  message += "\n-------------------------------------------------------------------";
                  System.err.println(message);
               }
            }
         }
      }
   }

   private static String toStringAsFramelessObjects(Object[] frameObjects)
   {
      String ret = "[";
      for (int i = 0; i < frameObjects.length; i++)
      {
         ret += toStringAsFramelessObject(frameObjects[i]);
         if (i < frameObjects.length - 1)
            ret += ", ";
         else
            ret += "]";
      }
      return ret;
   }

   private static String toStringAsFramelessObject(Object frameObject)
   {
      if (frameObject instanceof FrameGeometryObject)
         return ((FrameGeometryObject<?, ?>) frameObject).getGeometryObject().toString();
      else
         return frameObject.toString();
   }

   @SuppressWarnings("unchecked")
   private static <T extends GeometryObject<T>, S> boolean epsilonEquals(Object framelessParameter, Object frameParameter, double epsilon)
   {
      if (framelessParameter == null && frameParameter == null)
         return true;

      if (framelessParameter != null ^ frameParameter != null)
         return false;

      if (framelessParameter instanceof Clearable && frameParameter instanceof Clearable)
      {
         if (((Clearable) framelessParameter).containsNaN() && ((Clearable) frameParameter).containsNaN())
            return true;
      }

      if (isFramelessObject(framelessParameter))
      {
         if (!isFrameObject(frameParameter) && !isFramelessObject(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         if (isFrameObject(frameParameter))
            return ((EpsilonComparable<T>) framelessParameter).epsilonEquals(((FrameGeometryObject<?, T>) frameParameter).getGeometryObject(), epsilon);
         else
            return ((EpsilonComparable<T>) framelessParameter).epsilonEquals((T) frameParameter, epsilon);
      }

      if (isFrameObject(framelessParameter))
      {
         if (!isFrameObject(frameParameter) && !isFramelessObject(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         if (isFrameObject(frameParameter))
            return ((EpsilonComparable<T>) framelessParameter).epsilonEquals((T) frameParameter, epsilon);
         else
            return ((EpsilonComparable<T>) frameParameter).epsilonEquals(((FrameGeometryObject<?, T>) framelessParameter).getGeometryObject(), epsilon);
      }

      if (Double.TYPE.isInstance(framelessParameter) || Float.TYPE.isInstance(framelessParameter))
      {
         if (!Double.TYPE.isInstance(frameParameter) && !Float.TYPE.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return EuclidCoreTools.epsilonEquals((double) framelessParameter, (double) frameParameter, epsilon);
      }

      if (Integer.TYPE.isInstance(framelessParameter) || Long.TYPE.isInstance(framelessParameter))
      {
         if (!Integer.TYPE.isInstance(frameParameter) && !Long.TYPE.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return (long) framelessParameter == (long) frameParameter;
      }

      if (Double.class.isInstance(framelessParameter) || Float.class.isInstance(framelessParameter))
      {
         if (!Double.class.isInstance(frameParameter) && !Float.class.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         double framelessDouble = ((Number) framelessParameter).doubleValue();
         double frameDouble = ((Number) frameParameter).doubleValue();
         return Double.compare(framelessDouble, frameDouble) == 0 || EuclidCoreTools.epsilonEquals(framelessDouble, frameDouble, epsilon);
      }

      if (Integer.class.isInstance(framelessParameter) || Long.class.isInstance(framelessParameter))
      {
         if (!Integer.class.isInstance(frameParameter) && !Long.class.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return ((Number) framelessParameter).longValue() == ((Number) frameParameter).longValue();
      }

      if (Boolean.class.isInstance(framelessParameter))
      {
         if (!Boolean.class.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return (boolean) framelessParameter == (boolean) frameParameter;
      }

      if (framelessParameter instanceof EpsilonComparable && frameParameter instanceof EpsilonComparable)
      {
         return ((EpsilonComparable<S>) framelessParameter).epsilonEquals((S) frameParameter, epsilon);
      }

      if (framelessParameter instanceof List)
      {
         if (frameParameter instanceof List)
         {
            List<?> framelessList = (List<?>) framelessParameter;
            List<?> frameList = (List<?>) frameParameter;

            if (framelessList.size() != frameList.size())
               return false;

            for (int i = 0; i < framelessList.size(); i++)
            {
               if (!epsilonEquals(framelessList.get(i), frameList.get(i), epsilon))
                  return false;
            }
            return true;
         }
         else
         {
            throw new RuntimeException("Reached unexpected state.");
         }
      }

      if (framelessParameter instanceof DenseMatrix64F && frameParameter instanceof DenseMatrix64F)
      {
         return MatrixFeatures.isEquals((DenseMatrix64F) framelessParameter, (DenseMatrix64F) frameParameter, epsilon);
      }

      if (framelessParameter instanceof float[] && frameParameter instanceof float[])
      {
         float[] framelessArray = (float[]) framelessParameter;
         float[] frameArray = (float[]) frameParameter;

         if (framelessArray.length != frameArray.length)
            return false;
         for (int i = 0; i < framelessArray.length; i++)
         {
            if (Float.compare(framelessArray[i], frameArray[i]) != 0 && !EuclidCoreTools.epsilonEquals(framelessArray[i], frameArray[i], epsilon))
               return false;
         }
         return true;
      }

      if (framelessParameter instanceof double[] && frameParameter instanceof double[])
      {
         double[] framelessArray = (double[]) framelessParameter;
         double[] frameArray = (double[]) frameParameter;

         if (framelessArray.length != frameArray.length)
            return false;
         for (int i = 0; i < framelessArray.length; i++)
         {
            if (Double.compare(framelessArray[i], frameArray[i]) != 0 && !EuclidCoreTools.epsilonEquals(framelessArray[i], frameArray[i], epsilon))
               return false;
         }
         return true;
      }

      if (framelessParameter instanceof String && frameParameter instanceof String)
         return true;

      if (framelessParameter instanceof Class && frameParameter instanceof Class)
         return true;

      throw new RuntimeException("Did not expect the following types: " + framelessParameter.getClass().getSimpleName() + " & "
            + frameParameter.getClass().getSimpleName());
   }

   private static boolean isFrameTypeReadOnly(Class<?> frameType)
   {
      return frameReadOnlyTypes.contains(frameType) && !frameMutableTypes.contains(frameType);
   }

   private static boolean isFrameTypeMutable(Class<?> frameType)
   {
      return frameMutableTypes.contains(frameType);
   }

   private static Object invokeStaticMethod(Method frameMethod, Object[] parameters) throws Throwable
   {
      try
      {
         return frameMethod.invoke(null, parameters);
      }
      catch (IllegalAccessException | IllegalArgumentException e)
      {
         System.err.println("Something went wrong when invoking the static method: " + getMethodSimpleName(frameMethod));
         System.err.println("Objects used as parameters: " + getArgumentTypeString(parameters));
         e.printStackTrace();
         throw e;
      }
      catch (InvocationTargetException e)
      {
         throw e.getCause();
      }
   }

   private static Object invokeMethod(Object methodHolder, Method frameMethod, Object[] parameters) throws Throwable
   {
      try
      {
         return frameMethod.invoke(methodHolder, parameters);
      }
      catch (IllegalAccessException | IllegalArgumentException e)
      {
         System.err.println("Something went wrong when invoking the static method: " + getMethodSimpleName(frameMethod));
         System.err.println("Objects used as parameters: " + getArgumentTypeString(parameters));
         e.printStackTrace();
         throw e;
      }
      catch (InvocationTargetException e)
      {
         throw e.getCause();
      }
   }

   private static boolean isExceptionAcceptable(Throwable t)
   {
      return acceptableExceptions.stream().filter(c -> c.isAssignableFrom(t.getClass())).findAny().isPresent();
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
                                                                   Class<?>[] overloadingSignature, Class<?> typeToSearchIn)
         throws SecurityException
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
         throw new AssertionError("The original method in " + typeWithOriginalMethod.getSimpleName() + ":\n" + getMethodSimpleName(originalMethod) + "\nis not properly overloaded, expected to find in " + typeToSearchIn.getSimpleName() + ":\n"
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
      return ret.substring(1, ret.length() - 1);
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

   private static List<Method> keepOnlyMethodsWithAtLeastNFramelessArguments(Method[] methodsToFilter, int minNumberOfFramelessArguments)
   {
      return keepOnlyMethodsWithAtLeastNFramelessArguments(Arrays.asList(methodsToFilter), minNumberOfFramelessArguments);
   }

   private static List<Method> keepOnlyMethodsWithAtLeastNFramelessArguments(List<Method> methodsToFilter, int minNumberOfFramelessArguments)
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

   private static Class<?> findCorrespondingFramelessType(Class<?> frameType)
   {
      if (!isFrameType(frameType))
         throw new IllegalArgumentException("Cannot handle the following type: " + frameType.getSimpleName());

      Class<?> framelessType = null;

      for (Entry<Class<?>, Class<?>> entry : framelessTypesToFrameTypesTable.entrySet())
      {
         if (!entry.getValue().isAssignableFrom(frameType))
            continue;

         if (framelessType == null || framelessType.isAssignableFrom(entry.getKey()))
            framelessType = entry.getKey();
      }

      if (framelessType == null)
         throw new RuntimeException("Could not find the corresponding frameless type for: " + frameType.getSimpleName());

      return framelessType;
   }

   private static boolean isFrameObject(Object object)
   {
      return isFrameType(object.getClass());
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

   private static boolean isFramelessObject(Object object)
   {
      return isFramelessType(object.getClass());
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

   private static Object[] clone(Object[] parametersToClone)
   {
      Object[] clone = new Object[parametersToClone.length];

      for (int i = 0; i < parametersToClone.length; i++)
      {
         Class<? extends Object> parameterType = parametersToClone[i].getClass();

         if (parametersToClone[i] instanceof FrameGeometryObject)
         {
            set(clone[i] = createFrameObject(parameterType, null), parametersToClone[i]);
         }
         else if (parameterType.isPrimitive() || parametersToClone[i] instanceof Number || parametersToClone[i] instanceof Boolean)
         {
            clone[i] = parametersToClone[i];
         }
         else if (DenseMatrix64F.class.equals(parameterType))
         {
            clone[i] = new DenseMatrix64F((DenseMatrix64F) parametersToClone[i]);
         }
         else if (float[].class.equals(parameterType))
         {
            float[] arrayToClone = (float[]) parametersToClone[i];
            clone[i] = new float[arrayToClone.length];
            System.arraycopy(arrayToClone, 0, clone[i], 0, arrayToClone.length);
         }
         else if (double[].class.equals(parameterType))
         {
            double[] arrayToClone = (double[]) parametersToClone[i];
            clone[i] = new double[arrayToClone.length];
            System.arraycopy(arrayToClone, 0, clone[i], 0, arrayToClone.length);
         }
         else
         {
            try
            {
               clone[i] = newInstanceOf(parameterType);
               Method setter = parameterType.getMethod("set", parameterType);
               setter.invoke(clone[i], parametersToClone[i]);
            }
            catch (NoSuchMethodException | SecurityException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
            {
               throw new RuntimeException("Unhandled type: " + parameterType.getSimpleName());
            }
         }
      }

      return clone;
   }

   @SuppressWarnings("unchecked")
   private static <F extends FrameGeometryObject<F, G>, G extends GeometryObject<G>> void set(Object fToSet, Object fToRead)
   {
      ((F) fToSet).setIncludingFrame((F) fToRead);
   }

   private static Object[] instantiateParameterTypes(ReferenceFrame frame, Class<?>[] parameterTypes)
   {
      Object[] parameters = new Object[parameterTypes.length];
      for (int i = 0; i < parameterTypes.length; i++)
      {
         parameters[i] = instantiateParameterType(frame, parameterTypes[i]);
         if (parameters[i] == null)
            return null;
      }
      return parameters;
   }

   private static Object instantiateParameterType(ReferenceFrame frame, Class<?> parameterType)
   {
      Object object = createFrameObject(parameterType, frame);
      if (object != null)
         return object;
      object = createFramelessObject(parameterType);
      if (object != null)
         return object;
      return newInstanceOf(parameterType);
   }

   private static Object createFramelessObject(Class<?> type)
   {
      GenericTypeBuilder builder = null;
      Class<?> bestMatchingType = null;

      for (Entry<Class<?>, GenericTypeBuilder> entry : framelessTypeBuilders.entrySet())
      {
         if (!entry.getKey().isAssignableFrom(type))
            continue;

         if (bestMatchingType == null || bestMatchingType.isAssignableFrom(entry.getKey()))
         {
            bestMatchingType = entry.getKey();
            builder = entry.getValue();
         }
      }

      return builder == null ? null : builder.newInstance();
   }

   private static Object createFrameObject(Class<?> type, ReferenceFrame referenceFrame)
   {
      RandomFrameTypeBuilder<?> builder = null;
      Class<?> bestMatchingType = null;

      for (Entry<Class<?>, RandomFrameTypeBuilder<?>> entry : frameTypeBuilders.entrySet())
      {
         if (!entry.getKey().isAssignableFrom(type))
            continue;

         if (bestMatchingType == null || bestMatchingType.isAssignableFrom(entry.getKey()))
         {
            bestMatchingType = entry.getKey();
            builder = entry.getValue();
         }
      }

      return builder == null ? null : builder.newInstance(referenceFrame);
   }

   private static Object newInstanceOf(Class<?> type)
   {
      if (type.isPrimitive())
      {
         if (type.equals(Boolean.TYPE))
            return random.nextBoolean();
         else if (type.equals(Integer.TYPE) || type.equals(Character.TYPE) || type.equals(Long.TYPE))
            return random.nextInt(1000) - 500;
         else if (type.equals(Float.TYPE) || type.equals(Double.TYPE))
            return EuclidCoreRandomTools.generateRandomDouble(random, 10.0);
         else
            return 0;
      }

      if (Transform.class.equals(type))
         return EuclidCoreRandomTools.generateRandomAffineTransform(random);

      if (DenseMatrix64F.class.equals(type))
      {
         return RandomMatrices.createRandom(20, 20, random);
      }

      if (float[].class.equals(type))
      {
         float[] ret = new float[20];
         for (int i = 0; i < ret.length; i++)
            ret[i] = random.nextFloat();
         return ret;
      }

      if (double[].class.equals(type))
      {
         double[] ret = new double[20];
         for (int i = 0; i < ret.length; i++)
            ret[i] = random.nextDouble();
         return ret;
      }

      if (Collection.class.equals(type))
      {
         return null;
      }

      if (Object.class.equals(type))
      {
         return null;
      }

      try
      {
         return type.getConstructor().newInstance();
      }
      catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException | NoSuchMethodException
            | SecurityException e)
      {
         throw new RuntimeException("Could not instantiate an object of the type: " + type.getSimpleName());
      }
   }

   public static interface RandomFrameTypeBuilder<T extends ReferenceFrameHolder>
   {
      T newInstance(ReferenceFrame referenceFrame);
   }

   public static interface FrameTypeBuilder<T extends ReferenceFrameHolder>
   {
      T newInstance(ReferenceFrame referenceFrame, Object framelessObject);
   }

   public static interface GenericTypeBuilder
   {
      Object newInstance();
   }
}
