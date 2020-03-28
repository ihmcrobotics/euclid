package us.ihmc.euclid.referenceFrame.api;

import static us.ihmc.euclid.referenceFrame.api.MethodSignature.getMethodSimpleName;

import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Random;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector4DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollBasics;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.interfaces.Transform;

/**
 * This class provides tools that using reflection can perform a variety of comparison-based
 * assertions on a frame geometry given its corresponding frameless type.
 * <p>
 * These tools are still experimental and are improved through heavy internal usage for building
 * Euclid's test suite. The objective it to make this class usable for third party classes.
 * </p>
 * <p>
 * This class relies on several conventions including the following:
 * <ul>
 * <li>A class with a name ending with the keyword "ReadOnly" refers to a geometry type which cannot
 * be modified.
 * <li>A class with a name ending with the keyword "Basics" refers to a geometry type which can be
 * modified.
 * <li>A class with a name starting with the keyword "FixedFrame" refers to a geometry type defined
 * in a {@code ReferenceFrame} which cannot be modified.
 * <li>A class with a name starting with the keyword "Frame" refers to a geometry type defined in a
 * {@code ReferenceFrame} which can be modified.
 * <li>A class with a name that does not start with either "Frame" or "FixedFrame" is a geometry
 * without the information of the frame in which it is expressed.
 * <li>Any setter named "setIncludingFrame" sets all the values of the geometry and sets its
 * reference frame from one of the arguments.
 * <li>Any setter named "setMatchingFrame" sets all the values of the geometry without changing it
 * reference frame. The arguments should be expressed in the same reference frame, but that
 * reference frame can be different from the geometry declaring the method. If the reference frame
 * is different, a transformation is applied on either the arguments or the geometry (depending on
 * which provides a natural outcome) without modifying the arguments.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameAPITester
{
   private static final String READ_ONLY = "ReadOnly";
   private static final String BASICS = "Basics";
   private static final String FRAME = "Frame";
   private static final String FIXED_FRAME = "FixedFrame";

   private static final String MATCHING_FRAME = "MatchingFrame";
   private static final String INCLUDING_FRAME = "IncludingFrame";
   private static final String SET_MATCHING_FRAME = "set" + MATCHING_FRAME;
   private static final String SET_INCLUDING_FRAME = "set" + INCLUDING_FRAME;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean DEBUG = false;
   private final static Random random = new Random(345345);
   private final static double epsilon = 1.0e-12;

   private final static Set<Class<?>> framelessTypesWithoutFrameEquivalent = new HashSet<>();
   private final static Map<Class<?>, Class<?>> framelessTypesToFrameTypesTable = new HashMap<>();
   private final static Set<Class<?>> frameReadOnlyTypes = new HashSet<>();
   private final static Set<Class<?>> fixedFrameMutableTypes = new HashSet<>();
   private final static Set<Class<?>> mutableFrameMutableTypes = new HashSet<>();

   static
   {
      registerEuclidFrameTypes();
   }

   private static void registerEuclidFrameTypes()
   {
      ReflectionBasedBuilders.registerRandomGeneratorClasses(EuclidCoreRandomTools.class, EuclidGeometryRandomTools.class, EuclidFrameRandomTools.class);

      registerFrameTypesSmart(FrameTuple2DBasics.class, FrameVector2DBasics.class, FramePoint2DBasics.class);
      registerFrameTypesSmart(FrameTuple3DBasics.class, FrameVector3DBasics.class, FramePoint3DBasics.class);
      registerFrameTypesSmart(FrameTuple4DBasics.class, FrameVector4DBasics.class, FrameQuaternionBasics.class);
      registerFrameTypeSmart(FrameYawPitchRollBasics.class);
      registerFrameTypesSmart(FrameRotationMatrixBasics.class, FrameMatrix3DBasics.class);
      registerFrameTypesSmart(FrameOrientation2DBasics.class, FrameOrientation3DBasics.class);
      registerFrameTypesSmart(FramePose2DBasics.class, FramePose3DBasics.class);
      registerFrameTypesSmart(FrameLine2DBasics.class, FrameLine3DBasics.class);
      registerFrameTypesSmart(FrameLineSegment2DBasics.class, FrameLineSegment3DBasics.class);
      registerFrameTypesSmart(FrameConvexPolygon2DBasics.class);
      registerFrameTypesSmart(FrameBoundingBox2DBasics.class, FrameBoundingBox3DBasics.class);

      registerReadOnlyFrameTypeSmart(FrameVertex2DSupplier.class);
      registerReadOnlyFrameTypeSmart(FrameVertex3DSupplier.class);

      registerFramelessTypeSmart(AxisAngleBasics.class);
      registerFramelessType(RotationScaleMatrix.class, RotationScaleMatrixReadOnly.class);
   }

   private final static Set<Class<?>> acceptableExceptions = new HashSet<>();
   static
   {
      acceptableExceptions.add(BoundingBoxException.class);
      acceptableExceptions.add(IllegalArgumentException.class);
      acceptableExceptions.add(RuntimeException.class);
   }

   /**
    * Registers frameless types that do not have a frame type equivalent.
    * <p>
    * This is needed for preventing false assertion errors.
    * </p>
    * 
    * @param framelessMutableTypes the mutable API of the frameless geometry types to be registered.
    * @see #registerFramelessTypeSmart(Class)
    */
   public static void registerFramelessTypesSmart(Class<?>... framelessMutableTypes)
   {
      for (Class<?> framelessMutableType : framelessMutableTypes)
         registerFramelessTypeSmart(framelessMutableType);
   }

   /**
    * Registers a frameless type that does not have a frame type equivalent.
    * <p>
    * This is needed for preventing false assertion errors.
    * </p>
    * <p>
    * The read-only equivalent type is retrieved by using naming convention and registered.
    * </p>
    * 
    * @param framelessMutableType the mutable API of the frameless geometry type to be registered.
    */
   public static void registerFramelessTypeSmart(Class<?> framelessMutableType)
   {
      Class<?> framelessReadOnlyType = searchSuperInterfaceFromSimpleName(framelessMutableType.getSimpleName().replace(BASICS, READ_ONLY),
                                                                          framelessMutableType);

      Objects.requireNonNull(framelessReadOnlyType, "Could not find read-only type for " + framelessMutableType.getSimpleName());
      framelessTypesWithoutFrameEquivalent.addAll(Arrays.asList(framelessMutableType, framelessReadOnlyType));
   }

   /**
    * Registers the read-only and basics API for a frameless type that does not have a frame type
    * equivalent.
    * <p>
    * This is needed for preventing false assertion errors.
    * </p>
    * 
    * @param framelessMutableType  the mutable API of the frameless geometry type.
    * @param framelessReadOnlyType the read-only API of the frameless geometry type.
    */
   public static void registerFramelessType(Class<?> framelessMutableType, Class<?> framelessReadOnlyType)
   {
      framelessTypesWithoutFrameEquivalent.addAll(Arrays.asList(framelessMutableType, framelessReadOnlyType));
   }

   /**
    * Registers the given types as frame geometries which values and frame can be modified.
    * 
    * @param mutableFrameMutableTypes the types to be registered.
    * @see #registerFramelessTypeSmart(Class)
    */
   public static void registerFrameTypesSmart(Class<?>... mutableFrameMutableTypes)
   {
      for (Class<?> mutableFrameMutableType : mutableFrameMutableTypes)
         registerFrameTypeSmart(mutableFrameMutableType);
   }

   /**
    * Registers the given type as a frame geometry which values and frame can be modified.
    * <p>
    * <ul>
    * <li>The fixed-frame and read-only super types are retrieved by using naming convention and
    * registered.
    * <li>The mutable and read-only frameless equivalent types are retrieved by using naming convention
    * and registered.
    * </ul>
    * </p>
    * 
    * @param mutableFrameMutableType the geometry with: mutable values and mutable frame to be
    *                                registered.
    */
   public static void registerFrameTypeSmart(Class<?> mutableFrameMutableType)
   {
      String mutableFrameMutableTypeName = mutableFrameMutableType.getSimpleName();

      String fixedFrameMutableTypeName = mutableFrameMutableTypeName.replace(FRAME, FIXED_FRAME);
      Class<?> fixedFrameMutableType = searchSuperInterfaceFromSimpleName(fixedFrameMutableTypeName, mutableFrameMutableType);

      String frameReadOnlyTypeName = mutableFrameMutableTypeName.replace(BASICS, READ_ONLY);
      Class<?> frameReadOnlyType = searchSuperInterfaceFromSimpleName(frameReadOnlyTypeName, fixedFrameMutableType);

      String framelessMutableTypeName = mutableFrameMutableTypeName.replace(FRAME, "");
      Class<?> framelessMutableType = searchSuperInterfaceFromSimpleName(framelessMutableTypeName, fixedFrameMutableType);

      String framelessReadOnlyTypeName = framelessMutableType.getSimpleName().replace(BASICS, READ_ONLY);
      Class<?> framelessReadOnlyType = searchSuperInterfaceFromSimpleName(framelessReadOnlyTypeName, framelessMutableType);

      Objects.requireNonNull(fixedFrameMutableType, "Could not find fixed-frame mutable type for " + mutableFrameMutableType.getSimpleName());
      Objects.requireNonNull(frameReadOnlyType, "Could not find frame read-only type for " + mutableFrameMutableType.getSimpleName());
      Objects.requireNonNull(framelessMutableType, "Could not find frameless mutable type for " + mutableFrameMutableType.getSimpleName());
      Objects.requireNonNull(framelessReadOnlyType, "Could not find frameless read-only type for " + mutableFrameMutableType.getSimpleName());

      registerFrameType(mutableFrameMutableType, fixedFrameMutableType, frameReadOnlyType, framelessMutableType, framelessReadOnlyType);
   }

   /**
    * Registers the given types as representing the same geometry with varying attributes.
    * <p>
    * This tester requires all needed types to be registered before performing assertions.
    * </p>
    * 
    * @param mutableFrameMutableType the geometry with: mutable values and mutable frame.
    * @param fixedFrameMutableType   the geometry with: mutable values and immutable frame.
    * @param frameReadOnlyType       the geometry with: immutable values and immutable frame.
    * @param framelessMutableType    the geometry with: mutable values and no frame information.
    * @param framelessReadOnlyType   the geometry with: immutable values and no frame information.
    */
   public static void registerFrameType(Class<?> mutableFrameMutableType, Class<?> fixedFrameMutableType, Class<?> frameReadOnlyType,
                                        Class<?> framelessMutableType, Class<?> framelessReadOnlyType)
   {
      Objects.requireNonNull(framelessReadOnlyType, "Frameless read-only type cannot be null.");
      Objects.requireNonNull(frameReadOnlyType, "Frame read-only type cannot be null.");
      framelessTypesToFrameTypesTable.put(framelessReadOnlyType, frameReadOnlyType);
      Objects.requireNonNull(framelessMutableType, "Frameless mutable type cannot be null.");
      if (fixedFrameMutableType != null)
         framelessTypesToFrameTypesTable.put(framelessMutableType, fixedFrameMutableType);
      else if (mutableFrameMutableType != null)
         framelessTypesToFrameTypesTable.put(framelessMutableType, mutableFrameMutableType);
      else
         throw new NullPointerException("Either fixedFrameMutableType or mutableFrameMutableType has to be not null.");

      frameReadOnlyTypes.add(frameReadOnlyType);

      if (fixedFrameMutableType != null)
         fixedFrameMutableTypes.add(fixedFrameMutableType);
      if (mutableFrameMutableType != null)
         mutableFrameMutableTypes.add(mutableFrameMutableType);
   }

   /**
    * Registers frame types that only declares read-only API.
    * 
    * @param frameReadOnlyTypes the read-only frame types to be registered.
    * @see #registerReadOnlyFrameTypeSmart(Class)
    */
   public static void registerReadOnlyFrameTypeSmart(Class<?>... frameReadOnlyTypes)
   {
      for (Class<?> frameReadOnlyType : frameReadOnlyTypes)
         registerReadOnlyFrameTypeSmart(frameReadOnlyType);
   }

   /**
    * Registers a frame type that only declares read-only API.
    * <p>
    * The read-only frameless equivalent type is retrieved by using naming convention and registered.
    * </p>
    * 
    * @param frameReadOnlyType the read-only frame type to be registered.
    */
   public static void registerReadOnlyFrameTypeSmart(Class<?> frameReadOnlyType)
   {
      Class<?> framelessReadOnlyType = searchSuperInterfaceFromSimpleName(frameReadOnlyType.getSimpleName().replace(FRAME, ""), frameReadOnlyType);
      Objects.requireNonNull(framelessReadOnlyType, "Could not find frameless read-only type for " + frameReadOnlyType.getSimpleName());

      framelessTypesToFrameTypesTable.put(framelessReadOnlyType, frameReadOnlyType);
      frameReadOnlyTypes.add(frameReadOnlyType);
   }

   private EuclidFrameAPITester()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Asserts, using reflection, that all methods with frameless arguments, such as
    * {@code Tuple3DReadOnly}, are overloaded with their frame type equivalent, i.e.
    * {@code Tuple2DBasics} is to be overloaded with {@code FrameTuple2D}.
    *
    * @param typeWithFrameMethods     refers to the type to be tested. This asserts that
    *                                 {@code typeWithFrameMethods} properly has all the methods
    *                                 necessary to properly overload {@code typeWithFramelessMethods}.
    * @param typeWithFramelessMethods refers to the type declaring methods with frameless objects that
    *                                 are to be overloaded.
    * @param assertAllCombinations    when {@code false}, this asserts that for each method in
    *                                 {@code typeWithFramelessMethods} there is one overloading method
    *                                 in {@code typeWithFrameMethods} with all the arguments using the
    *                                 equivalent frame type. When {@code true}, this asserts that for
    *                                 each method in {@code typeWithFramelessArguments},
    *                                 {@code typeWithFrameMethods} overloads it with all the possible
    *                                 combinations of frame & frameless arguments, except for the
    *                                 original frameless signature.
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
    * @param typeWithFrameMethods          refers to the type to be tested. This asserts that
    *                                      {@code typeWithFrameMethods} properly has all the methods
    *                                      necessary to properly overload
    *                                      {@code typeWithFramelessMethods}.
    * @param typeWithFramelessMethods      refers to the type declaring methods with frameless objects
    *                                      that are to be overloaded.
    * @param assertAllCombinations         when {@code false}, this asserts that for each method in
    *                                      {@code typeWithFramelessMethods} there is one overloading
    *                                      method in {@code typeWithFrameMethods} with all the
    *                                      arguments using the equivalent frame type. When
    *                                      {@code true}, this asserts that for each method in
    *                                      {@code typeWithFramelessArguments},
    *                                      {@code typeWithFrameMethods} overloads it with all the
    *                                      possible combinations of frame & frameless arguments, except
    *                                      for the original frameless signature.
    * @param minNumberOfFramelessArguments threshold used to filter out methods to assert in
    *                                      {@code typeWithFramelessMethods}.
    */
   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods, boolean assertAllCombinations,
                                                        int minNumberOfFramelessArguments)
   {
      assertOverloadingWithFrameObjects(typeWithFrameMethods, typeWithFramelessMethods, assertAllCombinations, minNumberOfFramelessArguments, m -> true);
   }

   /**
    * Creates a filter that can be used to ignore the collection of methods as defined by the given
    * signatures.
    * 
    * @param signaturesToIgnore the signatures of the methods to be ignored.
    * @return the filter.
    */
   public static Predicate<Method> methodFilterFromSignature(Collection<MethodSignature> signaturesToIgnore)
   {
      List<Predicate<Method>> filters = signaturesToIgnore.stream().map(EuclidFrameAPITester::methodFilterFromSignature).collect(Collectors.toList());
      return method -> filters.stream().allMatch(filter -> filter.test(method));
   }

   /**
    * Creates a filter that can be used to ignore the method as defined by the given signature.
    * 
    * @param signatureToIgnore the signature of the method to be ignored.
    * @return the filter.
    */
   public static Predicate<Method> methodFilterFromSignature(MethodSignature signatureToIgnore)
   {
      return method ->
      {
         if (!signatureToIgnore.getName().equals(method.getName()))
            return true;
         if (Arrays.equals(method.getParameterTypes(), signatureToIgnore.toParameterTypeArray()))
            return false;
         else
            return true;
      };
   }

   /**
    * Asserts, using reflection, that all methods with frameless arguments, such as
    * {@code Tuple3DReadOnly}, are overloaded with their frame type equivalent, i.e.
    * {@code Tuple2DBasics} is to be overloaded with {@code FrameTuple2DBasics} and/or
    * {@code FixedFrameTuple2DBasics}.
    *
    * @param typeWithFrameMethods          refers to the type to be tested. This asserts that
    *                                      {@code typeWithFrameMethods} properly has all the methods
    *                                      necessary to properly overload
    *                                      {@code typeWithFramelessMethods}.
    * @param typeWithFramelessMethods      refers to the type declaring methods with frameless objects
    *                                      that are to be overloaded.
    * @param assertAllCombinations         when {@code false}, this asserts that for each method in
    *                                      {@code typeWithFramelessMethods} there is one overloading
    *                                      method in {@code typeWithFrameMethods} with all the
    *                                      arguments using the equivalent frame type. When
    *                                      {@code true}, this asserts that for each method in
    *                                      {@code typeWithFramelessArguments},
    *                                      {@code typeWithFrameMethods} overloads it with all the
    *                                      possible combinations of frame & frameless arguments, except
    *                                      for the original frameless signature.
    * @param minNumberOfFramelessArguments threshold used to filter out methods to assert in
    *                                      {@code typeWithFramelessMethods}.
    * @param framelessMethodFilter         custom filter used on the methods of
    *                                      {@code typeWithFramelessMethods}. The assertions are
    *                                      performed on the methods for which
    *                                      {@code framelessMethodFilter.test(method)} returns
    *                                      {@code true}.
    */
   public static void assertOverloadingWithFrameObjects(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods, boolean assertAllCombinations,
                                                        int minNumberOfFramelessArguments, Predicate<Method> framelessMethodFilter)
   {
      // The frame methods are all the methods from 'typeWithFramelessMethods' that have at least one geometry argument.
      Predicate<Method> filter = framelessMethodFilter.and(atLeastNFramelessParameters(minNumberOfFramelessArguments));
      List<MethodSignature> framelessSignatures = Stream.of(typeWithFramelessMethods.getMethods()).filter(filter).map(MethodSignature::new)
                                                        .collect(Collectors.toList());

      for (MethodSignature framelessSignature : framelessSignatures)
      {
         // Creating all the expected combinations
         List<MethodSignature> expectedMethodSignatures = createExpectedMethodSignaturesWithFrameArgument(framelessSignature, assertAllCombinations);

         for (MethodSignature expectedMethodSignature : expectedMethodSignatures)
         {
            assertMethodOverloadedWithSpecificSignature(typeWithFrameMethods, typeWithFramelessMethods, framelessSignature, expectedMethodSignature);
         }
      }
   }

   private static void assertMethodOverloadedWithSpecificSignature(Class<?> typeWithOverloadingMethods, Class<?> typeWithOriginalMethods,
                                                                   MethodSignature originalSignature, MethodSignature overloadingSignature)
         throws SecurityException
   {
      try
      {
         Method overloadingMethod = typeWithOverloadingMethods.getMethod(originalSignature.getName(), overloadingSignature.toParameterTypeArray());
         Class<?> originalReturnType = originalSignature.getReturnType();
         Class<?> overloadingReturnType = overloadingMethod.getReturnType();

         { // Assert the return type is proper
            if ((originalReturnType == null) != (overloadingReturnType == null))
            {
               String message = "Inconsistency found in the return type.";
               message += "\nOriginal    method: " + originalSignature.getMethodSimpleName();
               message += "\nOverloading method: " + getMethodSimpleName(overloadingMethod);
               message += "\nOriginal type declaring method: " + typeWithOriginalMethods.getSimpleName();
               message += "\nType overloading original     : " + typeWithOverloadingMethods.getSimpleName();
               throw new AssertionError(message);
            }

            if (overloadingReturnType.equals(originalReturnType) || overloadingReturnType == findCorrespondingFrameType(originalReturnType))
               return;

            if (overloadingReturnType.isAssignableFrom(findCorrespondingFrameType(originalReturnType)))
            {
               String message = "Unexpected return type: expected: " + findCorrespondingFrameType(originalReturnType).getSimpleName() + ", actual: "
                     + overloadingReturnType.getSimpleName();
               message += "\nOriginal    method: " + originalSignature.getMethodSimpleName();
               message += "\nOverloading method: " + getMethodSimpleName(overloadingMethod);
               message += "\nOriginal type declaring method: " + typeWithOriginalMethods.getSimpleName();
               message += "\nType overloading original     : " + typeWithOverloadingMethods.getSimpleName();
               throw new AssertionError(message);
            }
         }
      }
      catch (NoSuchMethodException e)
      {
         throw new AssertionError("The original method in " + typeWithOriginalMethods.getSimpleName() + ":\n" + originalSignature.getMethodSimpleName()
               + "\nis not properly overloaded, expected to find in " + typeWithOverloadingMethods.getSimpleName() + ":\n"
               + overloadingSignature.getMethodSimpleName());
      }
   }

   /**
    * Asserts API convention for setMatchingFrame: for a setter like
    * {@code FramelessType.set(Point2DReadOnly, Tuple3DReadOnly)} the following setters should be
    * declared in the frame type:
    * <ul>
    * <li>{@code FrameType.setMatchingFrame(ReferenceFrame, Point2DReadOnly, Tuple3DReadOnly)}
    * <li>{@code FrameType.setMatchingFrame(FramePoint2DReadOnly, FrameTuple3DReadOnly)}
    * </ul>
    *
    * @param typeWithFrameMethods     the frame type which API is to be tested.
    * @param typeWithFramelessMethods the frameless type used as reference.
    * @param framelessMethodFilter    custom filter used on the methods. The assertions are performed
    *                                 on the methods for which
    *                                 {@code framelessMethodFilter.test(method)} returns {@code true}.
    */
   public static void assertAPIDeclareMatchingFrameSetters(Class<?> typeWithFrameMethods, Class<?> typeWithFramelessMethods,
                                                           Predicate<Method> framelessMethodFilter)
   {
      Predicate<Method> filter = framelessMethodFilter.and(atLeastNFramelessParameters(1)).and(m -> m.getName().equals("set"));
      List<MethodSignature> framelessSignatures = Stream.of(typeWithFramelessMethods.getMethods()).filter(filter).map(MethodSignature::new)
                                                        .collect(Collectors.toList());

      for (MethodSignature framelessSetterSignature : framelessSignatures)
      {
         List<MethodSignature> expectedSetMatchingFrameSignatures = createExpectedSetMatchingFrameSignatures(framelessSetterSignature);

         for (MethodSignature expectedSetMatchingFrameSignature : expectedSetMatchingFrameSignatures)
         {
            try
            {
               Method setMatchingFrameMethod = typeWithFrameMethods.getMethod(expectedSetMatchingFrameSignature.getName(),
                                                                              expectedSetMatchingFrameSignature.toParameterTypeArray());

               Class<?> originalReturnType = framelessSetterSignature.getReturnType();
               Class<?> overloadingReturnType = setMatchingFrameMethod.getReturnType();

               { // Assert the return type is proper
                  if (originalReturnType == null && overloadingReturnType != null)
                  {
                     String message = "Inconsistency found in the return type.";
                     message += "\nOriginal setter: " + framelessSetterSignature.getMethodSimpleName();
                     message += "\nCorresponding setMatchingFrame: " + getMethodSimpleName(setMatchingFrameMethod);
                     message += "\nOriginal type declaring method: " + typeWithFramelessMethods.getSimpleName();
                     message += "\nType declaring setMatchingFrame: " + typeWithFrameMethods.getSimpleName();
                     throw new AssertionError(message);
                  }

                  if (overloadingReturnType.equals(originalReturnType))
                     return;

                  if (overloadingReturnType.isAssignableFrom(findCorrespondingFrameType(originalReturnType)))
                     throw new AssertionError("Unexpected return type: expected: " + findCorrespondingFrameType(originalReturnType).getSimpleName()
                           + ", actual: " + overloadingReturnType.getSimpleName());
               }
            }
            catch (NoSuchMethodException e)
            {
               throw new AssertionError("Could not find setMatchingFrame correspond to the original setter in " + typeWithFramelessMethods.getSimpleName()
                     + ":\n" + framelessSetterSignature.getMethodSimpleName() + "\nExpected to find in " + typeWithFrameMethods.getSimpleName() + ":\n"
                     + expectedSetMatchingFrameSignature.getMethodSimpleName());
            }
         }
      }
   }

   private static List<MethodSignature> createExpectedSetMatchingFrameSignatures(MethodSignature framelessSetterSignature)
   {
      assert framelessSetterSignature.getName().equals("set");

      List<MethodSignature> signatures = new ArrayList<>();
      // setMatchingFrame(ReferenceFrame, FramelessType(s))
      MethodSignature expectedSetMatchingFrameSignature = new MethodSignature(framelessSetterSignature);
      expectedSetMatchingFrameSignature.setName(SET_MATCHING_FRAME);
      expectedSetMatchingFrameSignature.addParameterType(0, ReferenceFrame.class);
      signatures.add(expectedSetMatchingFrameSignature);

      // setMatchingFrame(FrameType(s))
      expectedSetMatchingFrameSignature = new MethodSignature(framelessSetterSignature);
      expectedSetMatchingFrameSignature.setName(SET_MATCHING_FRAME);

      for (int i = 0; i < expectedSetMatchingFrameSignature.getParameterCount(); i++)
      {
         Class<?> parameterType = expectedSetMatchingFrameSignature.getParameterType(i);
         if (isFramelessTypeWithFrameEquivalent(parameterType))
            expectedSetMatchingFrameSignature.setParameterType(i, findCorrespondingFrameType(parameterType));
      }
      signatures.add(expectedSetMatchingFrameSignature);
      return signatures;
   }

   /**
    * Asserts that the method "setMatchingFrame" implementations are consistent with the regular
    * setters.
    * <p>
    * When the arguments are expressed in a different reference frame than the geometry on which the
    * method is called, a transformation as to be performed such that
    * {@code frameGeometry.setMatchingFrame(frameArgument)} is equivalent to either:
    * 
    * <pre>
    * ReferenceFrame originalFrame = frameGeometry.getReferenceFrame();
    * frameGeometry.setReferenceFrame(frameArgument.getReferenceFrame());
    * frameGeometry.set(frameArgument);
    * frameGeometry.changeFrame(originalFrame);
    * </pre>
    * 
    * or:
    * 
    * <pre>
    * ReferenceFrame originalFrame = frameArgument.getReferenceFrame();
    * frameArgument.changeFrame(frameGeometry.getReferenceFrame());
    * frameGeometry.set(frameArgument);
    * frameArgument.changeFrame(originalFrame);
    * </pre>
    * 
    * without modifying the arguments. Most of the time, the operation should be equivalent to the
    * first example, but for instance when {@code frameGeometry} is a 2D geometry and that
    * {@code frameArgument} is a 3D geometry, then the second option is expected as transforming the 2D
    * geometry would result in loss of information pre-transformation.
    * </p>
    * 
    * @param frameTypeBuilder   the builder for creating instances of the frame object to test.
    * @param numberOfIterations number of iterations to perform for each method.
    */
   public static void assertSetMatchingFramePreserveFunctionality(RandomFrameTypeBuilder frameTypeBuilder, int numberOfIterations)
   {
      assertSetMatchingFramePreserveFunctionality(frameTypeBuilder, m -> true, numberOfIterations);
   }

   /**
    * Asserts that the method "setMatchingFrame" implementations are consistent with the regular
    * setters.
    * <p>
    * When the arguments are expressed in a different reference frame than the geometry on which the
    * method is called, a transformation as to be performed such that
    * {@code frameGeometry.setMatchingFrame(frameArgument)} is equivalent to either:
    * 
    * <pre>
    * ReferenceFrame originalFrame = frameGeometry.getReferenceFrame();
    * frameGeometry.setReferenceFrame(frameArgument.getReferenceFrame());
    * frameGeometry.set(frameArgument);
    * frameGeometry.changeFrame(originalFrame);
    * </pre>
    * 
    * or:
    * 
    * <pre>
    * ReferenceFrame originalFrame = frameArgument.getReferenceFrame();
    * frameArgument.changeFrame(frameGeometry.getReferenceFrame());
    * frameGeometry.set(frameArgument);
    * frameArgument.changeFrame(originalFrame);
    * </pre>
    * 
    * without modifying the arguments. Most of the time, the operation should be equivalent to the
    * first example, but for instance when {@code frameGeometry} is a 2D geometry and that
    * {@code frameArgument} is a 3D geometry, then the second option is expected as transforming the 2D
    * geometry would result in loss of information pre-transformation.
    * </p>
    * 
    * @param frameTypeBuilder   the builder for creating instances of the frame object to test.
    * @param methodFilter       custom filter used on the methods. The assertions are performed on the
    *                           methods for which {@code methodFilter.test(method)} returns
    *                           {@code true}.
    * @param numberOfIterations number of iterations to perform for each method.
    */
   public static void assertSetMatchingFramePreserveFunctionality(RandomFrameTypeBuilder frameTypeBuilder, Predicate<Method> methodFilter,
                                                                  int numberOfIterations)
   {
      Class<? extends ReferenceFrameHolder> frameType = frameTypeBuilder.newInstance(random, worldFrame).getClass();

      Predicate<Method> filter = methodFilter.and(m -> m.getName().equals(SET_MATCHING_FRAME));
      List<Method> frameMethods = Stream.of(frameType.getMethods()).filter(filter).collect(Collectors.toList());

      for (Method matchingFrameMethod : frameMethods)
      {
         try
         {
            ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
            ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

            Method setterMethod = findCorrespondingSetterToSetMatchingIncludingFrame(frameType, matchingFrameMethod);

            int retryCounter = 0;

            for (int iteration = 0; iteration < numberOfIterations; iteration++)
            {
               Object[] matchingFrameMethodParameters = ReflectionBasedBuilders.next(random, frameA, matchingFrameMethod.getParameterTypes());

               Object[] setterMethodParameters = ReflectionBasedBuilders.clone(matchingFrameMethodParameters);
               if (setterMethodParameters == null)
               {
                  System.err.println("Cloning parameters failed for\n\t" + getMethodSimpleName(matchingFrameMethod) + "\n\tparameters: "
                        + getArgumentTypeString(matchingFrameMethodParameters));
                  retryCounter++;
                  if (retryCounter > 50)
                     throw new AssertionError("Retried too many times, aborting.");
                  else
                     System.out.println("Retrying.");
                  iteration--;
                  continue;
               }

               boolean isLastParameterToCheck2DTransform = is2DType(frameType)
                     && matchingFrameMethod.getParameterTypes()[matchingFrameMethod.getParameterCount() - 1] == boolean.class;

               if (matchingFrameMethod.getParameterTypes()[0] == ReferenceFrame.class)
               {
                  matchingFrameMethodParameters[0] = frameA;
                  setterMethodParameters = Arrays.copyOfRange(setterMethodParameters, 1, setterMethodParameters.length);
               }

               if (isLastParameterToCheck2DTransform)
               { // Last argument is "boolean checkIfTransformInXYPlane"
                  setterMethodParameters = Arrays.copyOfRange(setterMethodParameters, 0, setterMethodParameters.length - 1);
               }

               ReferenceFrameHolder matchingFrameObject = frameTypeBuilder.newInstance(random, frameB);
               ReferenceFrameHolder setterObject = frameTypeBuilder.newInstance(random, frameA);

               Throwable expectedException = null;
               Object setterMethodReturnObject = null;
               Object matchingFrameMethodReturnObject = null;

               try
               {
                  if (isLastParameterToCheck2DTransform)
                  {
                     setterMethodReturnObject = invokeMethod(setterObject, setterMethod, setterMethodParameters);
                     Method applyTransformMethod = frameType.getMethod("applyTransform", Transform.class, boolean.class);
                     invokeMethod(setterObject,
                                  applyTransformMethod,
                                  frameA.getTransformToDesiredFrame(frameB),
                                  matchingFrameMethodParameters[matchingFrameMethodParameters.length - 1]);
                     ((FrameChangeable) setterObject).setReferenceFrame(frameB);
                  }
                  else if (is2DType(frameType) && Stream.of(setterMethodParameters).map(Object::getClass).allMatch(EuclidFrameAPITester::is3DType))
                  { // The transformation should be done on the arguments not the holder.
                     setterObject = frameTypeBuilder.newInstance(random, frameB);

                     Object[] localSetterMethodParameters = ReflectionBasedBuilders.clone(setterMethodParameters);
                     if (setterMethodParameters == null)
                     {
                        System.err.println("Cloning parameters failed for\n\t" + getMethodSimpleName(matchingFrameMethod) + "\n\tparameters: "
                              + getArgumentTypeString(matchingFrameMethodParameters));
                        retryCounter++;
                        if (retryCounter > 50)
                           throw new AssertionError("Retried too many times, aborting.");
                        else
                           System.out.println("Retrying.");
                        iteration--;
                        continue;
                     }

                     for (int paramIndex = 0; paramIndex < localSetterMethodParameters.length; paramIndex++)
                     {
                        Object setterMethodParameter = localSetterMethodParameters[paramIndex];

                        if (setterMethodParameter instanceof FrameVertex3DSupplier)
                        {
                           FrameVertex3DSupplier asSupplier = (FrameVertex3DSupplier) setterMethodParameter;
                           List<FramePoint3D> vertices = new ArrayList<>();
                           for (int vertexIndex = 0; vertexIndex < asSupplier.getNumberOfVertices(); vertexIndex++)
                              vertices.add(new FramePoint3D(asSupplier.getVertex(vertexIndex)));
                           vertices.forEach(v -> v.changeFrame(frameB));
                           localSetterMethodParameters[paramIndex] = FrameVertex3DSupplier.asFrameVertex3DSupplier(vertices);
                        }
                        else
                        {
                           ((FrameChangeable) setterMethodParameter).changeFrame(frameB);
                        }
                     }
                     setterMethodReturnObject = invokeMethod(setterObject, setterMethod, localSetterMethodParameters);
                  }
                  else
                  {
                     setterMethodReturnObject = invokeMethod(setterObject, setterMethod, setterMethodParameters);
                     ((FrameChangeable) setterObject).changeFrame(frameB);
                  }
               }
               catch (Throwable e)
               {
                  expectedException = e;
               }

               try
               {
                  matchingFrameMethodReturnObject = invokeMethod(matchingFrameObject, matchingFrameMethod, matchingFrameMethodParameters);
               }
               catch (Throwable e)
               {
                  if (expectedException == null || e.getClass() != expectedException.getClass())
                  {
                     reportInconsistentException(matchingFrameMethod, setterMethod, expectedException, e);
                  }
                  else
                  {
                     continue;
                  }
               }

               int shift = matchingFrameMethod.getParameterTypes()[0] == ReferenceFrame.class ? 1 : 0;

               for (int i = 0; i < setterMethodParameters.length; i++)
               {
                  Object setterParameter = setterMethodParameters[i];
                  Object matchingFrameParameter = matchingFrameMethodParameters[i + shift];

                  if (!ReflectionBasedComparer.epsilonEquals(setterParameter, matchingFrameParameter, epsilon))
                     reportInconsistentArguments(matchingFrameMethod,
                                                 setterMethod,
                                                 matchingFrameMethodParameters,
                                                 setterMethodParameters,
                                                 setterParameter,
                                                 matchingFrameParameter);
               }

               if (!ReflectionBasedComparer.epsilonEquals(setterMethodReturnObject, matchingFrameMethodReturnObject, epsilon))
                  reportInconsistentReturnedType(matchingFrameMethod, setterMethod, setterMethodReturnObject, matchingFrameMethodReturnObject);

               if (!ReflectionBasedComparer.epsilonEquals(setterObject, matchingFrameObject, epsilon))
                  reportInconsistentObject(matchingFrameMethod, setterObject, matchingFrameObject, setterMethod);
            }
         }
         catch (RuntimeException e)
         {
            System.err.println("Problem when evaluating the method: " + getMethodSimpleName(matchingFrameMethod));
            throw e;
         }
      }
   }

   /**
    * Asserts that the method "setIncludingFrame" implementations are consistent with the regular
    * setters.
    * <p>
    * While "set" does not modify the frame of the holder, "setIncludingFrame" set the values AND the
    * frame of the holder.
    * </p>
    * 
    * @param frameTypeBuilder   the builder for creating instances of the frame object to test.
    * @param numberOfIterations number of iterations to perform for each method.
    */
   public static void assertSetIncludingFramePreserveFunctionality(RandomFrameTypeBuilder frameTypeBuilder, int numberOfIterations)
   {
      assertSetIncludingFramePreserveFunctionality(frameTypeBuilder, m -> true, numberOfIterations);
   }

   /**
    * Asserts that the method "setIncludingFrame" implementations are consistent with the regular
    * setters.
    * <p>
    * While "set" does not modify the frame of the holder, "setIncludingFrame" set the values AND the
    * frame of the holder.
    * </p>
    * 
    * @param frameTypeBuilder   the builder for creating instances of the frame object to test.
    * @param methodFilter       custom filter used on the methods. The assertions are performed on the
    *                           methods for which {@code methodFilter.test(method)} returns
    *                           {@code true}.
    * @param numberOfIterations number of iterations to perform for each method.
    */
   public static void assertSetIncludingFramePreserveFunctionality(RandomFrameTypeBuilder frameTypeBuilder, Predicate<Method> methodFilter,
                                                                   int numberOfIterations)
   {
      Class<? extends ReferenceFrameHolder> frameType = frameTypeBuilder.newInstance(random, worldFrame).getClass();

      Predicate<Method> filter = methodFilter.and(m -> m.getName().equals(SET_INCLUDING_FRAME));
      List<Method> frameMethods = Stream.of(frameType.getMethods()).filter(filter).collect(Collectors.toList());

      for (Method includingFrameMethod : frameMethods)
      {
         try
         {
            ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
            ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

            Method setterMethod = findCorrespondingSetterToSetMatchingIncludingFrame(frameType, includingFrameMethod);

            int retryCounter = 0;

            for (int iteration = 0; iteration < numberOfIterations; iteration++)
            {
               Object[] includingFrameMethodParameters = ReflectionBasedBuilders.next(random, frameA, includingFrameMethod.getParameterTypes());
               Object[] setterMethodParameters = ReflectionBasedBuilders.clone(includingFrameMethodParameters);
               if (setterMethodParameters == null)
               {
                  System.err.println("Cloning parameters failed for\n\t" + getMethodSimpleName(includingFrameMethod) + "\n\tparameters: "
                        + getArgumentTypeString(includingFrameMethodParameters));
                  retryCounter++;
                  if (retryCounter > 50)
                     throw new AssertionError("Retried too many times, aborting.");
                  else
                     System.out.println("Retrying.");
                  iteration--;
                  continue;
               }

               if (includingFrameMethod.getParameterTypes()[0] == ReferenceFrame.class)
               {
                  includingFrameMethodParameters[0] = frameA;
                  setterMethodParameters = Arrays.copyOfRange(setterMethodParameters, 1, setterMethodParameters.length);
               }

               ReferenceFrameHolder includingFrameObject = frameTypeBuilder.newInstance(random, frameB);
               ReferenceFrameHolder setterObject = frameTypeBuilder.newInstance(random, frameA);

               Throwable expectedException = null;
               Object setterMethodReturnObject = null;
               Object includingFrameMethodReturnObject = null;

               try
               {
                  setterMethodReturnObject = invokeMethod(setterObject, setterMethod, setterMethodParameters);
               }
               catch (Throwable e)
               {
                  expectedException = e;
               }

               try
               {
                  includingFrameMethodReturnObject = invokeMethod(includingFrameObject, includingFrameMethod, includingFrameMethodParameters);
               }
               catch (Throwable e)
               {
                  if (expectedException == null || e.getClass() != expectedException.getClass())
                  {
                     reportInconsistentException(includingFrameMethod, setterMethod, expectedException, e);
                  }
                  else
                  {
                     continue;
                  }
               }

               int shift = includingFrameMethod.getParameterTypes()[0] == ReferenceFrame.class ? 1 : 0;

               for (int i = 0; i < setterMethodParameters.length; i++)
               {
                  Object setterParameter = setterMethodParameters[i];
                  Object matchingFrameParameter = includingFrameMethodParameters[i + shift];

                  if (!ReflectionBasedComparer.epsilonEquals(setterParameter, matchingFrameParameter, epsilon))
                     reportInconsistentArguments(includingFrameMethod,
                                                 setterMethod,
                                                 includingFrameMethodParameters,
                                                 setterMethodParameters,
                                                 setterParameter,
                                                 matchingFrameParameter);
               }

               if (!ReflectionBasedComparer.epsilonEquals(setterMethodReturnObject, includingFrameMethodReturnObject, epsilon))
                  reportInconsistentReturnedType(includingFrameMethod, setterMethod, setterMethodReturnObject, includingFrameMethodReturnObject);

               if (!ReflectionBasedComparer.epsilonEquals(setterObject, includingFrameObject, epsilon))
                  reportInconsistentObject(includingFrameMethod, setterObject, includingFrameObject, setterMethod);
            }
         }
         catch (RuntimeException e)
         {
            System.err.println("Problem when evaluating the method: " + getMethodSimpleName(includingFrameMethod));
            throw e;
         }
      }
   }

   private static Method findCorrespondingSetterToSetMatchingIncludingFrame(Class<?> frameType, Method setMatchingIncludingFrameMethod)
   {
      MethodSignature frameSetterSignature = new MethodSignature(setMatchingIncludingFrameMethod);
      frameSetterSignature.setName("set");

      // The following is for setMatchingFrame for 2D frame objects.
      Class<?> lastParameter = frameSetterSignature.getParameterType(setMatchingIncludingFrameMethod.getParameterCount() - 1);

      if (lastParameter == boolean.class && is2DType(frameType))
      {
         frameSetterSignature.removeParameterType(frameSetterSignature.getParameterCount() - 1);
      }

      if (frameSetterSignature.getParameterType(0) == ReferenceFrame.class)
      {
         frameSetterSignature.removeParameterType(0);
      }

      try
      {
         return frameType.getMethod(frameSetterSignature.getName(), frameSetterSignature.toParameterTypeArray());
      }
      catch (NoSuchMethodException e)
      {
         throw new AssertionError("Could not find the frameless setter that corresponds to :\n" + getMethodSimpleName(setMatchingIncludingFrameMethod)
               + ", declared in " + setMatchingIncludingFrameMethod.getDeclaringClass().getSimpleName() + "\nExpected to find in " + frameType.getSimpleName()
               + ":\n" + frameSetterSignature.getMethodSimpleName());
      }
   }

   /**
    * Asserts, using reflection, that the methods, that are public and static, in
    * {@code typeHoldingStaticMethodsToTest} are properly checking and/or setting reference frames of
    * their arguments.
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
    * @param numberOfIterations               number of iterations to perform for each method.
    * @throws Throwable if an unexpected throwable has been thrown by a method at invocation time.
    */
   public static void assertStaticMethodsCheckReferenceFrame(Class<?> typeDeclaringStaticMethodsToTest, int numberOfIterations) throws Throwable
   {
      assertStaticMethodsCheckReferenceFrame(typeDeclaringStaticMethodsToTest, m -> true, numberOfIterations);
   }

   /**
    * Asserts, using reflection, that the methods, that are public and static, in
    * {@code typeHoldingStaticMethodsToTest} are properly checking and/or setting reference frames of
    * their arguments.
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
    * @param methodFilter                     custom filter used on the methods. The assertions are
    *                                         performed on the methods for which
    *                                         {@code methodFilter.test(method)} returns {@code true}.
    * @param numberOfIterations               number of iterations to perform for each method.
    * @throws Throwable if an unexpected throwable has been thrown by a method at invocation time.
    */
   public static void assertStaticMethodsCheckReferenceFrame(Class<?> typeDeclaringStaticMethodsToTest, Predicate<Method> methodFilter, int numberOfIterations)
         throws Throwable
   {
      Predicate<Method> filter = methodFilter.and(atLeastNFrameParameters(2)).and(m -> Modifier.isStatic(m.getModifiers()))
                                             .and(m -> Modifier.isPublic(m.getModifiers()));
      List<Method> frameMethods = Stream.of(typeDeclaringStaticMethodsToTest.getMethods()).filter(filter).collect(Collectors.toList());
      // Methods returning a frame type
      List<Method> methodsWithReturnFrameType = frameMethods.stream().filter(m -> isFrameType(m.getReturnType())).collect(Collectors.toList());

      for (int iteration = 0; iteration < numberOfIterations; iteration++)
      {
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

         // First check that the method is fine with all the arguments in the same frame.
         for (Method frameMethod : frameMethods)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
               if (!isFrameOfFrameTypeMutable(parameterType))
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
                  boolean mutateFrame = !isFrameOfFrameTypeMutable(parameterType);

                  if (!mutateFrame)
                  {
                     parameters[j] = ReflectionBasedBuilders.next(random, frameA, parameterType);
                  }
                  else
                  {
                     ReferenceFrame frame = frameA;
                     int mask = (int) Math.pow(2, currentByte);
                     if ((i & mask) != 0)
                        frame = frameB;
                     parameters[j] = ReflectionBasedBuilders.next(random, frame, parameterType);
                     currentByte++;
                  }
               }

               try
               {
                  invokeStaticMethod(frameMethod, parameters);
                  failToThrowReferenceFrameMismatchException(typeDeclaringStaticMethodsToTest, frameMethod, parameters);
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
         for (Method frameMethod : frameMethods)
         {
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               if (isMutableFrameMutableType(parameterType))
                  parameters[i] = ReflectionBasedBuilders.next(random, frameB, parameterType);
               else
                  parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
               if (isMutableFrameMutableType(parameterType))
               {
                  ReferenceFrame newFrame = ((ReferenceFrameHolder) parameters[i]).getReferenceFrame();
                  if (newFrame != frameA)
                     failToChangeParameterFrame(typeDeclaringStaticMethodsToTest, frameMethod, parameters, i);
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
               parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
               failToSetResultFrame(typeDeclaringStaticMethodsToTest, frameMethod, parameters, result);
         }
      }
   }

   /**
    * Asserts, using reflection, that the methods, that are public and non-static, in the created
    * instance from {@code frameTypeBuilder} are properly checking and/or setting reference frames of
    * their arguments.
    * <p>
    * This assertion expects methods to be declaring arguments as read-only to inform that they are
    * used as input only, and as mutable to inform that they are the output(s).
    * </p>
    * <p>
    * This expects methods to throw a {@link ReferenceFrameMismatchException} to indicate that the
    * operation cannot be performed because at least one argument with an immutable frame is expressed
    * in a different reference frame.
    * </p>
    *
    * @param frameTypeBuilder   builder used to generate an instance of the type to be tested.
    * @param methodFilter       custom filter used on the methods. The assertions are performed on the
    *                           methods for which {@code methodFilter.test(method)} returns
    *                           {@code true}.
    * @param numberOfIterations number of iterations to perform for each method.
    * @throws Throwable if an unexpected throwable has been thrown by a method at invocation time.
    */
   public static void assertMethodsOfReferenceFrameHolderCheckReferenceFrame(RandomFrameTypeBuilder frameTypeBuilder, Predicate<Method> methodFilter,
                                                                             int numberOfIterations)
         throws Throwable
   {
      Class<? extends ReferenceFrameHolder> frameType = frameTypeBuilder.newInstance(random, worldFrame).getClass();

      Predicate<Method> filter = methodFilter.and(m -> Modifier.isPublic(m.getModifiers())).and(atLeastNFrameParameters(1));

      List<Method> frameMethods = Stream.of(frameType.getMethods()).filter(filter).collect(Collectors.toList());
      // Methods returning a frame type
      List<Method> methodsWithReturnFrameType = frameMethods.stream().filter(m -> isFrameType(m.getReturnType())).collect(Collectors.toList());

      for (int iteration = 0; iteration < numberOfIterations; iteration++)
      {
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);

         // First check that the method is fine with the holder and all the arguments in the same frame.
         for (Method frameMethod : frameMethods)
         {
            ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(random, frameA);
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
            if (frameMethod.getName().endsWith(MATCHING_FRAME))
            {
               assertSetMatchingFrameChecksFrames(frameTypeBuilder, frameMethod, frameA, frameB);
               continue;
            }
            else if (frameMethod.getName().endsWith(INCLUDING_FRAME))
            {
               assertSetIncludingFrameChecksFrames(frameTypeBuilder, frameMethod, frameA, frameB);
               continue;
            }

            ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(random, frameA);
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();

            int numberOfArgumentsToTest = 0;
            for (Class<?> parameterType : parameterTypes)
            {
               if (!isFrameOfFrameTypeMutable(parameterType))
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
                  boolean mutateFrame = !isFrameOfFrameTypeMutable(parameterType);

                  if (!mutateFrame)
                  {
                     parameters[j] = ReflectionBasedBuilders.next(random, frameA, parameterType);
                  }
                  else
                  {
                     ReferenceFrame frame = frameA;
                     int mask = (int) Math.pow(2, currentByte);
                     if ((i & mask) != 0)
                        frame = frameB;
                     parameters[j] = ReflectionBasedBuilders.next(random, frame, parameterType);
                     currentByte++;
                  }
               }

               try
               {
                  invokeMethod(frameObject, frameMethod, parameters);
                  failToThrowReferenceFrameMismatchException(frameType, frameMethod, parameters);
               }
               catch (ReferenceFrameMismatchException e)
               {
                  // Good
               }
               catch (Throwable t)
               {
                  if (!(t instanceof ReferenceFrameMismatchException))
                     failToThrowReferenceFrameMismatchException(frameType, frameMethod, parameters, t);
               }
            }
         }

         // Check that the frame of each mutable is changed (optional)
         for (Method frameMethod : frameMethods)
         {
            ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(random, frameA);
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               if (isMutableFrameMutableType(parameterType))
                  parameters[i] = ReflectionBasedBuilders.next(random, frameB, parameterType);
               else
                  parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
               if (isMutableFrameMutableType(parameterType))
               {
                  ReferenceFrame newFrame = ((ReferenceFrameHolder) parameters[i]).getReferenceFrame();
                  if (newFrame != frameA)
                     failToChangeParameterFrame(frameType, frameMethod, parameters, i);
               }
            }
         }

         // Check for methods returning a frame type that the reference frame is properly set.
         for (Method frameMethod : methodsWithReturnFrameType)
         {
            ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(random, frameA);
            Class<?>[] parameterTypes = frameMethod.getParameterTypes();
            Object[] parameters = new Object[parameterTypes.length];

            for (int i = 0; i < parameterTypes.length; i++)
            {
               Class<?> parameterType = parameterTypes[i];
               parameters[i] = ReflectionBasedBuilders.next(random, frameA, parameterType);
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
               failToSetResultFrame(frameType, frameMethod, parameters, result);
         }
      }
   }

   private static void assertSetIncludingFrameChecksFrames(RandomFrameTypeBuilder frameTypeBuilder, Method setIncludingFrameMethod, ReferenceFrame frameA,
                                                           ReferenceFrame frameB)
   { // For reference frame check, it should behave the same as setMatchingFrame.
      assertSetMatchingFrameChecksFrames(frameTypeBuilder, setIncludingFrameMethod, frameA, frameB);
   }

   private static void assertSetMatchingFrameChecksFrames(RandomFrameTypeBuilder frameTypeBuilder, Method setMatchingFrameMethod, ReferenceFrame frameA,
                                                          ReferenceFrame frameB)
   {
      ReferenceFrameHolder frameObject = frameTypeBuilder.newInstance(random, frameA);
      Class<? extends ReferenceFrameHolder> frameType = frameObject.getClass();
      Class<?>[] parameterTypes = setMatchingFrameMethod.getParameterTypes();

      for (int i = 0; i < setMatchingFrameMethod.getParameterCount(); i++)
      {
         Class<?> parameterType = parameterTypes[i];
         if (isFrameType(parameterType))
         {
            if (!frameReadOnlyTypes.contains(parameterType))
            {
               String message = setMatchingFrameMethod.getName() + " is expected to only request read-only parameters.\n";
               message += "In " + frameType.getSimpleName() + " the " + i + "th parameter is not a read-only:\n";
               message += getMethodSimpleName(setMatchingFrameMethod);
               throw new AssertionError(message);
            }
         }
      }

      int numberOfArgumentsToTest = countFrameParameters(setMatchingFrameMethod);
      if (numberOfArgumentsToTest < 2)
      { // If there's less than 2 frame parameters, no reference frame check can be done.
         return;
      }

      int numberOfCombinations = (int) Math.pow(2, numberOfArgumentsToTest);

      for (int i = 1; i < numberOfCombinations - 1; i++)
      {
         Object[] parameters = new Object[parameterTypes.length];
         int currentByte = 0;

         for (int j = 0; j < parameterTypes.length; j++)
         {
            Class<?> parameterType = parameterTypes[j];

            if (isFrameType(parameterType))
            {
               ReferenceFrame frame = frameA;
               int mask = (int) Math.pow(2, currentByte);
               if ((i & mask) != 0)
                  frame = frameB;
               parameters[j] = ReflectionBasedBuilders.next(random, frame, parameterType);
               currentByte++;
            }
            else
            {
               parameters[j] = ReflectionBasedBuilders.next(random, frameA, parameterType);
            }
         }

         try
         {
            invokeMethod(frameObject, setMatchingFrameMethod, parameters);
            failToThrowReferenceFrameMismatchException(frameType, setMatchingFrameMethod, parameters);
         }
         catch (ReferenceFrameMismatchException e)
         {
            // Good
         }
         catch (Throwable t)
         {
            if (!isExceptionAcceptable(t))
               failToThrowReferenceFrameMismatchException(frameType, setMatchingFrameMethod, parameters, t);
         }
      }
   }

   /**
    * Assuming the type {@code typeWithFrameMethodsToTest} declares the same static methods as declared
    * in {@code typeWithFramlessMethods} with the difference of dealing with reference frame holders,
    * this method asserts that the methods in {@code typeWithFrameMethodsToTest} does not change the
    * underlying algorithms.
    * <p>
    * For each method declared in {@code typeWithFrameMethodsToTest}, this methods searched for the
    * equivalent method in {@code typeWithFramelessMethods} and the methods from both classes are
    * invoked to compare the output.
    * </p>
    *
    * @param typeWithFrameMethodsToTest the type in which the methods are to be tested.
    * @param typeWithFramelessMethods   the type declaring the methods against which the methods from
    *                                   {@code typeWithFrameMethodsToTest} are to be compared.
    * @param numberOfIterations         number of iterations to perform for each method.
    */
   public static void assertStaticMethodsPreserveFunctionality(Class<?> typeWithFrameMethodsToTest, Class<?> typeWithFramelessMethods, int numberOfIterations)
   {
      assertStaticMethodsPreserveFunctionality(typeWithFrameMethodsToTest, typeWithFramelessMethods, m -> true, numberOfIterations);
   }

   /**
    * Assuming the type {@code typeWithFrameMethodsToTest} declares the same static methods as declared
    * in {@code typeWithFramlessMethods} with the difference of dealing with reference frame holders,
    * this method asserts that the methods in {@code typeWithFrameMethodsToTest} does not change the
    * underlying algorithms.
    * <p>
    * For each method declared in {@code typeWithFrameMethodsToTest}, this methods searched for the
    * equivalent method in {@code typeWithFramelessMethods} and the methods from both classes are
    * invoked to compare the output.
    * </p>
    *
    * @param typeWithFrameMethodsToTest the type in which the methods are to be tested.
    * @param typeWithFramelessMethods   the type declaring the methods against which the methods from
    *                                   {@code typeWithFrameMethodsToTest} are to be compared.
    * @param methodFilter               custom filter used on the methods. The assertions are performed
    *                                   on the methods for which {@code methodFilter.test(method)}
    *                                   returns {@code true}.
    * @param numberOfIterations         number of iterations to perform for each method.
    */
   public static void assertStaticMethodsPreserveFunctionality(Class<?> typeWithFrameMethodsToTest, Class<?> typeWithFramelessMethods,
                                                               Predicate<Method> methodFilter, int numberOfIterations)
   {
      List<Method> frameMethods = Stream.of(typeWithFrameMethodsToTest.getMethods()).filter(methodFilter).collect(Collectors.toList());

      for (Method frameMethod : frameMethods)
      {
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

         int retryCounter = 0;

         for (int iteration = 0; iteration < numberOfIterations; iteration++)
         {
            try
            {
               Method framelessMethod = typeWithFramelessMethods.getMethod(frameMethodName, framelessMethodParameterTypes);
               Object[] frameMethodParameters = ReflectionBasedBuilders.next(random, worldFrame, frameMethodParameterTypes);
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

               Object[] framelessMethodParameters = ReflectionBasedBuilders.clone(frameMethodParameters);
               if (framelessMethodParameters == null)
               {
                  System.err.println("Cloning parameters failed for\n\t" + getMethodSimpleName(frameMethod) + "\n\tparameters: "
                        + getArgumentTypeString(frameMethodParameters));
                  retryCounter++;
                  if (retryCounter > 50)
                     throw new AssertionError("Retried too many times, aborting.");
                  else
                     System.out.println("Retrying.");
                  iteration--;
                  continue;
               }
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
                  if (expectedException == null || e.getClass() != expectedException.getClass())
                  {
                     reportInconsistentException(frameMethod, framelessMethod, expectedException, e);
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

                  if (!ReflectionBasedComparer.epsilonEquals(framelessParameter, frameParameter, epsilon))
                     reportInconsistentArguments(frameMethod,
                                                 framelessMethod,
                                                 frameMethodParameters,
                                                 framelessMethodParameters,
                                                 framelessParameter,
                                                 frameParameter);
               }

               if (!ReflectionBasedComparer.epsilonEquals(framelessMethodReturnObject, frameMethodReturnObject, epsilon))
                  reportInconsistentReturnedType(frameMethod, framelessMethod, framelessMethodReturnObject, frameMethodReturnObject);
            }
            catch (NoSuchMethodException e)
            {
               debugNoSuchMethodException(typeWithFrameMethodsToTest, typeWithFramelessMethods, frameMethod, framelessMethodParameterTypes);
            }
            catch (SecurityException e)
            {
               debugSecurityException(typeWithFramelessMethods, frameMethodName, framelessMethodParameterTypes);
            }
         }
      }
   }

   /**
    * Assuming the type built by the {@code frameTypeBuilder} declares the same methods as declared in
    * the type built by {@code framelessTypeBuilder} with the difference of handling the reference
    * frame information, this method asserts that the methods the type built by the
    * {@code frameTypeBuilder} does not change the underlying algorithms.
    * <p>
    * For each method declared in the type built by the {@code frameTypeBuilder}, this methods searched
    * for the equivalent method in type built by the {@code framelessTypeBuilder} and the methods from
    * both classes are invoked to compare the output.
    * </p>
    *
    * @param frameTypeCopier      the builder for creating instances of the frame object to test.
    * @param framelessTypeBuilber the builder for creating instances of the corresponding frameless
    *                             objects.
    * @param methodFilter         custom filter used on the methods. The assertions are performed on
    *                             the methods for which {@code methodFilter.test(method)} returns
    *                             {@code true}.
    * @param numberOfIterations   number of iterations to perform for each method.
    */
   public static void assertFrameMethodsOfFrameHolderPreserveFunctionality(FrameTypeCopier frameTypeCopier, RandomFramelessTypeBuilder framelessTypeBuilber,
                                                                           Predicate<Method> methodFilter, int numberOfIterations)
   {

      Class<? extends ReferenceFrameHolder> frameTypeToTest = frameTypeCopier.newInstance(worldFrame, framelessTypeBuilber.newInstance(random)).getClass();
      Class<? extends Object> framelessType = framelessTypeBuilber.newInstance(random).getClass();

      List<Method> frameMethods = Stream.of(frameTypeToTest.getMethods()).filter(methodFilter).collect(Collectors.toList());

      for (Method frameMethod : frameMethods)
      {
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

         int retryCounter = 0;

         for (int iteration = 0; iteration < numberOfIterations; iteration++)
         {
            Object framelessObject = framelessTypeBuilber.newInstance(random);
            ReferenceFrameHolder frameObject = frameTypeCopier.newInstance(worldFrame, framelessObject);

            try
            {
               Method framelessMethod = framelessType.getMethod(frameMethodName, framelessMethodParameterTypes);
               Object[] frameMethodParameters = ReflectionBasedBuilders.next(random, worldFrame, frameMethodParameterTypes);

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

               Object[] framelessMethodParameters = ReflectionBasedBuilders.clone(frameMethodParameters);
               if (framelessMethodParameters == null)
               {
                  System.err.println("Cloning parameters failed for\n\t" + getMethodSimpleName(frameMethod) + "\n\tparameters: "
                        + getArgumentTypeString(frameMethodParameters));
                  retryCounter++;
                  if (retryCounter > 50)
                     throw new AssertionError("Retried too many times, aborting.");
                  else
                     System.out.println("Retyring.");
                  iteration--;
                  continue;
               }
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
                  if (expectedException == null || e.getClass() != expectedException.getClass())
                  {
                     reportInconsistentException(frameMethod, framelessMethod, expectedException, e);
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

                  if (!ReflectionBasedComparer.epsilonEquals(framelessParameter, frameParameter, epsilon))
                     reportInconsistentArguments(frameMethod,
                                                 framelessMethod,
                                                 frameMethodParameters,
                                                 framelessMethodParameters,
                                                 framelessParameter,
                                                 frameParameter);
               }

               if (!ReflectionBasedComparer.epsilonEquals(framelessMethodReturnObject, frameMethodReturnObject, epsilon))
                  reportInconsistentReturnedType(frameMethod, framelessMethod, framelessMethodReturnObject, frameMethodReturnObject);

               if (!ReflectionBasedComparer.epsilonEquals(framelessObject, frameObject, epsilon))
                  reportInconsistentObject(frameMethod, framelessObject, frameObject, framelessMethod);
            }
            catch (NoSuchMethodException e)
            {
               debugNoSuchMethodException(frameTypeToTest, framelessType, frameMethod, framelessMethodParameterTypes);
            }
            catch (SecurityException e)
            {
               debugSecurityException(framelessType, frameMethodName, framelessMethodParameterTypes);
            }
            catch (RuntimeException e)
            {
               System.err.println("Problem when evaluating the method: "
                     + getMethodSimpleName(frameMethod.getReturnType(), frameMethodName, frameMethodParameterTypes));
               throw e;
            }
         }
      }
   }

   private static void failToSetResultFrame(Class<?> typeDeclaringMethod, Method frameMethod, Object[] parameters, Object result) throws AssertionError
   {
      String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not set the frame of the result.";
      message += "\nType being tested: " + typeDeclaringMethod.getSimpleName();
      message += "\nArguments used: " + Arrays.toString(parameters);
      message += "\nArgument types: " + getArgumentTypeString(parameters);
      message += "\nResult: " + result;
      throw new AssertionError(message);
   }

   private static void failToChangeParameterFrame(Class<?> typeDeclaringMethod, Method frameMethod, Object[] parameters, int parameterIndex)
         throws AssertionError
   {
      String message = "The method: " + getMethodSimpleName(frameMethod) + "\ndid not change the frame of the " + (parameterIndex + 1) + "th parameter.";
      message += "\nType being tested: " + typeDeclaringMethod.getSimpleName();
      message += "\nArguments used: " + Arrays.toString(parameters);
      message += "\nArgument types: " + getArgumentTypeString(parameters);
      throw new AssertionError(message);
   }

   private static void failToThrowReferenceFrameMismatchException(Class<?> typeDeclaringMethod, Method frameMethod, Object[] parameters) throws AssertionError
   {
      failToThrowReferenceFrameMismatchException(typeDeclaringMethod, frameMethod, parameters, null);
   }

   private static void failToThrowReferenceFrameMismatchException(Class<?> typeDeclaringMethod, Method frameMethod, Object[] parameters,
                                                                  Throwable exceptionThrownInstead)
         throws AssertionError
   {
      String message = "Should have thrown a " + ReferenceFrameMismatchException.class.getSimpleName();
      message += "\nType being tested: " + typeDeclaringMethod.getSimpleName();
      message += "\nMethod: " + getMethodSimpleName(frameMethod);
      message += "\nArguments used: " + Arrays.toString(parameters);
      message += "\nArgument types: " + getArgumentTypeString(parameters);
      if (exceptionThrownInstead != null)
         throw new AssertionError(message, exceptionThrownInstead);
      else
         throw new AssertionError(message);
   }

   private static void debugSecurityException(Class<?> typeWithFramelessMethods, String frameMethodName, Class<?>[] framelessMethodParameterTypes)
   {
      if (DEBUG)
      {
         String message = "";
         message += "-------------------------------------------------------------------";
         message += "\nUnable to access method with name: " + frameMethodName + " and argument types: " + getSimpleNames(framelessMethodParameterTypes);
         message += "\nin type: " + typeWithFramelessMethods.getSimpleName();
         message += "\n-------------------------------------------------------------------";
         System.err.println(message);
      }
   }

   private static void debugNoSuchMethodException(Class<?> typeWithFrameMethodsToTest, Class<?> typeWithFramelessMethods, Method frameMethod,
                                                  Class<?>[] framelessMethodParameterTypes)
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

   private static void reportInconsistentObject(Method frameMethod, Object framelessObject, ReferenceFrameHolder frameObject, Method framelessMethod)
         throws AssertionError
   {
      String message = "";
      message += "Detected a method inconsistent with its original method.";
      message += "\nInconsistent method: " + getMethodSimpleName(frameMethod);
      message += "\nOriginal     method: " + getMethodSimpleName(framelessMethod);
      message += "\nActual   object after method call:" + frameObject;
      message += "\nExpected object after method call:" + framelessObject;
      throw new AssertionError(message);
   }

   private static void reportInconsistentException(Method frameMethod, Method framelessMethod, Throwable expectedException, Throwable e) throws AssertionError
   {
      String message = "";
      message += "The method: " + getMethodSimpleName(frameMethod);
      message += "\ndid not throw the same exception as the original method: " + getMethodSimpleName(framelessMethod);
      message += "\nExpected exception class: " + (expectedException == null ? "none" : expectedException.getClass().getSimpleName());
      message += "\nActual   exception class: " + e.getClass().getSimpleName();
      throw new AssertionError(message);
   }

   private static void reportInconsistentArguments(Method frameMethod, Method framelessMethod, Object[] frameMethodParameters,
                                                   Object[] framelessMethodParameters, Object framelessParameter, Object frameParameter)
         throws AssertionError
   {
      String message = "";
      message += "Detected a method inconsistent with its original method.";
      message += "\nInconsistent method: " + getMethodSimpleName(frameMethod);
      message += "\nOriginal     method: " + getMethodSimpleName(framelessMethod);
      message += "\nActual   arguments after call:\n" + Arrays.toString(frameMethodParameters);
      message += "\nExpected arguments after call:\n"
            + EuclidCoreIOTools.getCollectionString("[", "]", ", ", Arrays.asList(framelessMethodParameters), EuclidFrameAPITester::toStringAsFramelessObject);
      throw new AssertionError(message);
   }

   private static void reportInconsistentReturnedType(Method frameMethod, Method framelessMethod, Object framelessMethodReturnObject,
                                                      Object frameMethodReturnObject)
         throws AssertionError
   {
      String message = "";
      message += "Detected a method inconsistent with its original method.";
      message += "\nInconsistent method: " + getMethodSimpleName(frameMethod);
      message += "\nOriginal     method: " + getMethodSimpleName(framelessMethod);
      message += "\nActual   method returned:" + frameMethodReturnObject;
      message += "\nExpected method returned:" + toStringAsFramelessObject(framelessMethodReturnObject);
      throw new AssertionError(message);
   }

   private static String toStringAsFramelessObject(Object frameObject)
   {
      if (isFrameObject(frameObject))
         return findCorrespondingFramelessType(frameObject.getClass()).cast(frameObject).toString();
      else
         return frameObject.toString();
   }

   private static boolean isMutableFrameMutableType(Class<?> frameType)
   {
      return mutableFrameMutableTypes.contains(frameType) && !fixedFrameMutableTypes.contains(frameType);
   }

   private static boolean isFrameOfFrameTypeMutable(Class<?> frameType)
   {
      return !fixedFrameMutableTypes.contains(frameType) && !frameReadOnlyTypes.contains(frameType);
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

   private static Object invokeMethod(Object methodHolder, Method frameMethod, Object... parameters) throws Throwable
   {
      try
      {
         return frameMethod.invoke(methodHolder, parameters);
      }
      catch (IllegalAccessException | IllegalArgumentException e)
      {
         System.err.println("Something went wrong when invoking the method: " + getMethodSimpleName(frameMethod));
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

   private static String getArgumentTypeString(Object... arguments)
   {
      return EuclidCoreIOTools.getCollectionString(", ", Arrays.asList(arguments), o -> o.getClass().getSimpleName());
   }

   private static String getSimpleNames(Class<?>[] types)
   {
      String ret = Arrays.stream(types).map(t -> t.getSimpleName()).collect(Collectors.toList()).toString();
      return ret.substring(1, ret.length() - 1);
   }

   private static Predicate<Method> atLeastNFrameParameters(int minNumberOfFrameParameters)
   {
      return method -> countFrameParameters(method) >= minNumberOfFrameParameters;
   }

   private static int countFrameParameters(Method method)
   {
      return (int) Stream.of(method.getParameterTypes()).filter(EuclidFrameAPITester::isFrameType).count();
   }

   private static Predicate<Method> atLeastNFramelessParameters(int minNumberOfFramelessParameters)
   {
      return method -> countFramelessParameters(method) >= minNumberOfFramelessParameters;
   }

   private static int countFramelessParameters(Method method)
   {
      return (int) Stream.of(method.getParameterTypes()).filter(EuclidFrameAPITester::isFramelessType).count();
   }

   private static List<MethodSignature> createExpectedMethodSignaturesWithFrameArgument(MethodSignature framelessSignature, boolean createAllCombinations)
   {
      List<MethodSignature> expectedFrameSignatures = new ArrayList<>();

      if (!createAllCombinations)
      {
         MethodSignature combination = new MethodSignature(framelessSignature);

         for (int k = 0; k < combination.getParameterCount(); k++)
         {
            if (isFramelessTypeWithFrameEquivalent(combination.getParameterType(k)))
               combination.setParameterType(k, findCorrespondingFrameType(combination.getParameterType(k)));
         }
         expectedFrameSignatures.add(combination);
      }
      else
      {
         int numberOfArgumentsToOverload = (int) Arrays.stream(framelessSignature.toParameterTypeArray()).filter(t -> isFramelessTypeWithFrameEquivalent(t))
                                                       .count();
         int numberOfCombinations = (int) Math.pow(2, numberOfArgumentsToOverload);

         for (int i = 0; i < numberOfCombinations; i++)
         {
            MethodSignature combination = new MethodSignature(framelessSignature);
            int currentByte = 0;

            for (int k = 0; k < combination.getParameterCount(); k++)
            {
               if (isFramelessTypeWithFrameEquivalent(combination.getParameterType(k)))
               {
                  int mask = (int) Math.pow(2, currentByte);
                  if ((i & mask) != 0)
                     combination.setParameterType(k, findCorrespondingFrameType(combination.getParameterType(k)));
                  currentByte++;
               }
            }
            expectedFrameSignatures.add(combination);
         }

         // Remove the original method from the combinations
         expectedFrameSignatures = expectedFrameSignatures.stream().filter(signature -> !signature.equals(framelessSignature)).collect(Collectors.toList());
      }
      return expectedFrameSignatures;
   }

   private static Class<?> findCorrespondingFrameType(Class<?> framelessType)
   {
      if (framelessType.isArray())
         return Array.newInstance(findCorrespondingFrameType(framelessType.getComponentType()), 0).getClass();

      if (!isFramelessTypeWithFrameEquivalent(framelessType))
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
      if (frameType.isArray())
         return Array.newInstance(findCorrespondingFramelessType(frameType.getComponentType()), 0).getClass();

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

   private static boolean isFramelessTypeWithFrameEquivalent(Class<?> framelessType)
   {
      if (framelessType.isArray())
         return isFramelessTypeWithFrameEquivalent(framelessType.getComponentType());
      return isFramelessType(framelessType) && !framelessTypesWithoutFrameEquivalent.contains(framelessType);
   }

   private static boolean is2DType(Class<?> type)
   {
      return findCorrespondingFramelessType(type).getSimpleName().contains("2D");
   }

   private static boolean is3DType(Class<?> type)
   {
      return findCorrespondingFramelessType(type).getSimpleName().contains("3D");
   }

   private static Class<?> searchSuperInterfaceFromSimpleName(String name, Class<?> typeToStartFrom)
   {
      for (Class<?> superInterface : typeToStartFrom.getInterfaces())
      {
         if (superInterface.getSimpleName().equals(name))
         {
            return superInterface;
         }
      }

      for (Class<?> superInterface : typeToStartFrom.getInterfaces())
      {
         Class<?> thoroughSearchResult = searchSuperInterfaceFromSimpleName(name, superInterface);
         if (thoroughSearchResult != null)
            return thoroughSearchResult;
      }
      return null;
   }
}
