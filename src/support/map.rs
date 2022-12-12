/// Маппинг диопазонов
/// *-------х-------*
/// ^min    ^v      ^max
/// percent = (v - min) / (max - min)
///
/// *-------x-------*
/// ^left   ^res    ^right
/// res = left + (right - left) * percent
pub fn map(v: f32, min: f32, max: f32, left: f32, right: f32) -> f32 {
    let percent = (v - min) / (max - min);

    let mapped_len = num::abs(right - left) * percent;
    if left < right {
        left + mapped_len
    } else {
        right + mapped_len
    }
}
