function [pdf] = norm_pdf(x, mu, sigma)
    % Vzorec pro hustotu pravdepodobnosti
    pdf = (1 / (sigma * sqrt(2 * pi))) * exp(-0.5 * ((x - mu) / sigma).^2);
end