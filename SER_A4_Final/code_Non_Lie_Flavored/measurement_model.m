function y_predict = measurement_model(translation, fu, fv, cu, cv, b)
    temp_trans = zeros(4,1);
    temp_intrics = zeros(4,1);

    temp_trans(1,1) = fu*translation(1,1);
    temp_trans(2,1) = fv*translation(2,1);
    temp_trans(3,1) = fu*(translation(1,1) - b);
    temp_trans(4,1) = fv*translation(2,1);

    temp_intrics(1,1) = cu;
    temp_intrics(2,1) = cv;
    temp_intrics(3,1) = cu;
    temp_intrics(4,1) = cv;

    y_predict = (1.0/translation(3,1)) * temp_trans + temp_intrics;
end